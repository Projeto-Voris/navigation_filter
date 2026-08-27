#!/usr/bin/env python3

import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn, LifecycleState
from rclpy.duration import Duration
from rclpy.time import Time

from nav_msgs.msg import Odometry
from marine_acoustic_msgs.msg import Dvl
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, TwistStamped, TransformStamped

import tf2_ros
from tf2_ros import TransformException

from include.imu_model import IMUModel
from include.twist_model import TwistModel
from include.pose_model import PoseModel
from include.nav_filter import NavFilter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# SÓ PARA NÃO DAR CONFLITO NA HORA DE USAR BAGS
sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE,
)

class NavFilterNode(LifecycleNode):

    def __init__(self):
        super().__init__('nav_filter')

        # --- tf2 buffer/listener ---
        self.tf_buffer_ = tf2_ros.Buffer()
        self.tf_listener_ = tf2_ros.TransformListener(self.tf_buffer_, self)
        self.declare_parameter('use_sim_time', True)

        # --- Declara parâmetros (equivalente ao declare_parameter do rclcpp) ---
        self.declare_parameter('base_link', 'base_link')

        self.declare_parameter('imu.link', 'base_link')
        self.declare_parameter('imu.accelerometer_random_walk_bias', [0.0000001, 0.0000001, 0.0000001])
        self.declare_parameter('imu.gyroscope_random_walk_bias', [0.0000001, 0.0000001, 0.0000001])
        self.declare_parameter('imu.accelerometer_noise', [0.0000001, 0.0000001, 0.0000001])
        self.declare_parameter('imu.gyroscope_noise', [0.0000001, 0.0000001, 0.0000001])
        self.declare_parameter('imu.correlation_noise', [0.0000001, 0.0000001, 0.0000001])
        self.declare_parameter('imu.correlation_matrix.row0', [100.0, 0.0, 0.0])
        self.declare_parameter('imu.correlation_matrix.row1', [0.0, 100.0, 0.0])
        self.declare_parameter('imu.correlation_matrix.row2', [0.0, 0.0, 100.0])
        self.declare_parameter('imu.update_time', 0.0001)

        self.declare_parameter('twist.link', 'base_link')
        self.declare_parameter('twist.noise', [0.01, 0.01, 0.01, 0.01, 0.01, 0.01])
        self.declare_parameter('twist.update_time', 0.0001)

        self.declare_parameter('pose.link', 'base_link')
        self.declare_parameter('pose.noise', [0.01, 0.01, 0.01, 0.01, 0.01, 0.01])
        self.declare_parameter('pose.update_time', 0.0001)

        # --- Publisher (lifecycle-managed, equivalente ao create_publisher do rclcpp_lifecycle) ---
        self.publisher_ = self.create_lifecycle_publisher(Odometry, '/nav_filter/odom', 10)

        # --- Subscriptions ---
        self.imu_subscription_ = self.create_subscription(Imu, '/mavros/imu/data_raw', self.imu_callback, sensor_qos)
        self.pose_stamped_subscription_ = self.create_subscription(PoseStamped, '/mavros/vision_pose/pose', self.pose_callback, sensor_qos)
        self.twist_stamped_subscription_ = self.create_subscription(Dvl, '/waterlinked_dvl_driver/velocity_report', self.dvl_callback, sensor_qos)

        # --- Estado interno ---
        self.filter_ = NavFilter()
        self.latest_imu_msg_ = None

        self.base_link_ = 'base_link'
        self.imu_link_ = 'base_link'
        self.twist_link_ = 'base_link'
        self.pose_link_ = 'base_link'

        self.imu_transform_ = None
        self.twist_transform_ = None
        self.pose_transform_ = None

    # ------------------------------------------------------------------
    # Callbacks do lifecycle
    # ------------------------------------------------------------------

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring Navigation Filter')
        try:
            self.base_link_ = self.get_parameter('base_link').value
            self.imu_link_ = self.get_parameter('imu.link').value

            acc_bias = self.get_parameter('imu.accelerometer_random_walk_bias').value
            gyro_bias = self.get_parameter('imu.gyroscope_random_walk_bias').value
            acc_noise = self.get_parameter('imu.accelerometer_noise').value
            gyro_noise = self.get_parameter('imu.gyroscope_noise').value
            corr_noise = self.get_parameter('imu.correlation_noise').value
            corr_row_0 = self.get_parameter('imu.correlation_matrix.row0').value
            corr_row_1 = self.get_parameter('imu.correlation_matrix.row1').value
            corr_row_2 = self.get_parameter('imu.correlation_matrix.row2').value
            imu_update_time = float(self.get_parameter('imu.update_time').value)

            self.twist_link_ = self.get_parameter('twist.link').value
            twist_noise = self.get_parameter('twist.noise').value
            twist_update_time = float(self.get_parameter('twist.update_time').value)

            self.pose_link_ = self.get_parameter('pose.link').value
            pose_noise = self.get_parameter('pose.noise').value
            pose_update_time = float(self.get_parameter('pose.update_time').value)

            # lookup_transform(target_frame, source_frame, time)
            self.imu_transform_ = self.tf_buffer_.lookup_transform(
                self.base_link_, self.imu_link_, Time())
            self.twist_transform_ = self.tf_buffer_.lookup_transform(
                self.imu_link_, self.twist_link_, Time())
            self.pose_transform_ = self.tf_buffer_.lookup_transform(
                self.imu_link_, self.pose_link_, Time())

            imu_transform_matrix = self.get_matrix_from_tf(self.imu_transform_)
            twist_transform_matrix = self.get_matrix_from_tf(self.twist_transform_)
            pose_transform_matrix = self.get_matrix_from_tf(self.pose_transform_)
    
            corr_matrix = np.array([
                [corr_row_0[0], corr_row_0[1], corr_row_0[2]],
                [corr_row_1[0], corr_row_1[1], corr_row_1[2]],
                [corr_row_2[0], corr_row_2[1], corr_row_2[2]],
            ], dtype=np.float64)

            imu_model = IMUModel(imu_transform_matrix, acc_bias, gyro_bias,
                                  acc_noise, gyro_noise, corr_noise, corr_matrix,
                                  imu_update_time)
            twist_model = TwistModel(twist_transform_matrix, twist_noise, twist_update_time)
            pose_model = PoseModel(pose_transform_matrix, pose_noise, pose_update_time)

            self.filter_ = NavFilter(imu_model, twist_model, pose_model)

            self.get_logger().info(f'Base: {self.base_link_}')
            self.get_logger().info(f'IMU: {self.imu_link_}')
            self.get_logger().info(f'Twist: {self.twist_link_}')
            self.get_logger().info(f'Pose: {self.pose_link_}')

            self.get_logger().info('Navigation Filter Configured')

        except TransformException as ex:
            self.get_logger().warn(f'Could not get transform: {ex}')
            return TransitionCallbackReturn.FAILURE

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Activating Navigation Filter')
        # equivalente a rclcpp::Rate rate(0.5); rate.sleep();  -> 2 segundos
        self.get_clock().sleep_for(Duration(seconds=2.0))

        self.get_logger().info('Navigation Filter Activated')
        return super().on_activate(state)

    # ------------------------------------------------------------------
    # Callbacks dos sensores
    # ------------------------------------------------------------------

    def imu_callback(self, msg: Imu):
        self.latest_imu_msg_ = msg

        if not self._is_active():
            return

        q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        euler_angles = Rotation.from_quat(q).as_euler('xyz')

        imu_measurement = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z,
            euler_angles[0], euler_angles[1], euler_angles[2],
        ], dtype=np.float32)

        self.filter_.update_imu(imu_measurement)
        self.get_logger().info('Imu publishing')

        odom_msg = self.state_to_odom(self.filter_.get_state())
        self.publisher_.publish(odom_msg)

    def pose_callback(self, msg: PoseStamped):
        if not self._is_active():
            return

        q = [msg.pose.orientation.x, msg.pose.orientation.y,
             msg.pose.orientation.z, msg.pose.orientation.w]
        euler_angles = Rotation.from_quat(q).as_euler('xyz')
        self.get_logger().info('Pose publishing')

        pose_measurement = np.array([
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
            euler_angles[0], euler_angles[1], euler_angles[2],
        ], dtype=np.float32)

        self.filter_.update_pose(pose_measurement)
        odom_msg = self.state_to_odom(self.filter_.get_state())
        self.publisher_.publish(odom_msg)

    def dvl_callback(self, msg: Dvl):
        if not self._is_active():
            return

        self.get_logger().info('DVL publishing')

        twist_measurement = np.array([
            msg.velocity.x, msg.velocity.y, msg.velocity.z,
            0.0, 0.0, 0.0,
        ], dtype=np.float32)

        self.filter_.update_twist(twist_measurement)
        odom_msg = self.state_to_odom(self.filter_.get_state())
        self.publisher_.publish(odom_msg)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _is_active(self) -> bool:
        return self._state_machine.current_state[1] == 'active'

    def state_to_odom(self, state) -> Odometry:
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self.base_link_
        odom_msg.child_frame_id = self.imu_link_

        odom_msg.pose.pose.position.x = float(state.position_[0])
        odom_msg.pose.pose.position.y = float(state.position_[1])
        odom_msg.pose.pose.position.z = float(state.position_[2])

        odom_msg.twist.twist.linear.x = float(state.velocity_[0])
        odom_msg.twist.twist.linear.y = float(state.velocity_[1])
        odom_msg.twist.twist.linear.z = float(state.velocity_[2])

        r = Rotation.from_euler(
            'xyz',
            [state.orientation_[0], state.orientation_[1], state.orientation_[2]])
        qx, qy, qz, qw = r.as_quat()

        odom_msg.pose.pose.orientation.x = float(qx)
        odom_msg.pose.pose.orientation.y = float(qy)
        odom_msg.pose.pose.orientation.z = float(qz)
        odom_msg.pose.pose.orientation.w = float(qw)

        return odom_msg

    def get_matrix_from_tf(self, tf: TransformStamped) -> np.ndarray:
        transform = np.zeros((4, 4), dtype=np.float32)
        transform[0, 3] = tf.transform.translation.x
        transform[1, 3] = tf.transform.translation.y
        transform[2, 3] = tf.transform.translation.z
        transform[3, 3] = 1.0

        q = [tf.transform.rotation.x, tf.transform.rotation.y,
             tf.transform.rotation.z, tf.transform.rotation.w]
        rotation = Rotation.from_quat(q).as_matrix().astype(np.float32)

        camera_to_base = np.eye(3, dtype=np.float32)
        rotation = camera_to_base @ rotation

        transform[0:3, 0:3] = rotation
        return transform


def main(args=None):
    rclpy.init(args=args)
    node = NavFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()