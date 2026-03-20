#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "dvl_msgs/msg/dvl.hpp"
#include "nav_filter.hpp"
#include "imu_model.hpp"

using namespace std::chrono_literals;
using rclcpp_lifecycle::LifecycleNode;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;



class NavFilterNode : public LifecycleNode
{
  public:
    NavFilterNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : LifecycleNode("nav_filter", options),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_)
    {
      /* Declare sensors parameters and create subscriptions */
      
      this->declare_parameter<std::string>("base_link", "base_link");

      /* Publish a erro msg */

      publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/ekf_erro", 10);
      
      /* Receives a imu, pose and twist stamped */

      // IMU IMU.msgs Input ---> angular orientations, velocities and linear accelerations
      kalman_filter_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/mavros/imu/data_raw", 10, std::bind(&NavFilterNode::kalman_filter_callback, this, std::placeholders::_1));
      // SLAM PoseStamped Input ---> position and orientation       
      imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/pose", 10, std::bind(&NavFilterNode::pose_callback, this, std::placeholders::_1));
      // DVL DVL.msgs Input ---> linear velocities
      mavros_subscription_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "/mavros/local_position/velocity_body", 10, std::bind(&NavFilterNode::dvl_callback, this, std::placeholders::_1));
    
    }

    /* Here we have the core callbacks that configure and activate the node.
    * those are essencial parts in the ros2 lifecycle node structure */

  protected:
    CallbackReturn on_configure(const rclcpp_lifecycle::State &)
    { 
      RCLCPP_INFO(get_logger(), "Configuring Navigation Filter");
      try
      {
        rclcpp::Time now = this->get_clock()->now();
        

      /* On configure --> declare parameters, get transforms and initialize  filter */

        this->get_parameter("base_link", base_link_);
        
        RCLCPP_INFO(get_logger(), "EKF ERRO Configured");
      }
      catch(const tf2::TransformException & ex)
      {
        RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
        return CallbackReturn::FAILURE;
      }

      return CallbackReturn::SUCCESS;
    }


    /* Callback that activates the node */

    CallbackReturn on_activate(const rclcpp_lifecycle::State &)
    {
      RCLCPP_INFO(this->get_logger(), "Activating EKF erro");
      rclcpp::Rate rate(0.5); // 0.5 Hz (2 seconds delay)
      rate.sleep();

      RCLCPP_INFO(this->get_logger(), "EKF erro Activated");
      return CallbackReturn::SUCCESS;
    }


    /* Callbacks that recive data from sensors and update the filter
    * We have acess because we declare this callbacks in the "subscribers"
    * this means that everytime this node recives information, it will
    * be stored in this callbacks                                      */


  private:

    // Declarar a msg de IMU mais recente, para compor o update_twist dentro do dvl_callback: 
    sensor_msgs::msg::Imu latest_imu_msg_;

    void imu_callback(const sensor_msgs::msg::Imu & msg)
    {
      latest_imu_msg_ = msg;

      if (this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
      {
        // Convert IMU message to Eigen vector
        Eigen::Matrix<float, 6, 1> imu_measurement;

        // Convert angular velocity from quaternion to Euler angles
        Eigen::Quaternionf q(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
        Eigen::Vector3f euler_angles = q.toRotationMatrix().eulerAngles(0, 1, 2);

        // Monta uma matriz 6x1 com as medições de aceleração linear e ângulos de Euler
        imu_measurement << msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z,
               euler_angles[0], euler_angles[1], euler_angles[2];

        // Update the filter with the IMU measurement
        filter_.update_imu(imu_measurement);

        // Convert the current state to Odometry message --> deve pegar os dados que consegue com IMU e realocar eles em formato de Odom.msg
        nav_msgs::msg::Odometry odom_msg = state_to_odom(filter_.get_state());
        publisher_->publish(odom_msg);
      }
    }

    void kalman_filter_callback(const nav_msgs::msg::Odometry & msg)
    {
      if (this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
      {
        Eigen::Quaternionf q(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);
        Eigen::Vector3f euler_angles = q.toRotationMatrix().eulerAngles(0, 1, 2);

        filter_.update_pose(Eigen::Matrix<float, 6, 1>(
          msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
          euler_angles[0], euler_angles[1], euler_angles[2]));
      }
    }


    // Dentro do dvl callback na real as únicas informações que realmente vao ser usadas são as velocidades lineares
    // No twist model, as velocidades angulares são todas zeradas
    void dvl_callback(const geometry_msgs::msg::TwistStamped & msg)
    {
      if (this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
      {
        // Update the filter with the DVL linear velocity and IMU angular velocity measurements
        filter_.update_twist(Eigen::Matrix<float, 6, 1>(
          msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z,
          0, 0, 0));
      }
    }

    // Convert State to Odometry message --> basicamente compor a mensagem de odometria a partir do estado estimado pela IMU, mas não uma msgs de Odom completa, só meio preenchida
    nav_msgs::msg::Odometry state_to_odom(State state)
    {
      nav_msgs::msg::Odometry odom_msg;
      odom_msg.header.stamp = this->now();
      odom_msg.header.frame_id = base_link_;
      odom_msg.child_frame_id = imu_link_;

      // Fill position and velocity
      odom_msg.pose.pose.position.x = state.position_[0];
      odom_msg.pose.pose.position.y = state.position_[1];
      odom_msg.pose.pose.position.z = state.position_[2];

      odom_msg.twist.twist.linear.x = state.velocity_[0];
      odom_msg.twist.twist.linear.y = state.velocity_[1];
      odom_msg.twist.twist.linear.z = state.velocity_[2];

      // Convert orientation from Euler angles to quaternion
      Eigen::Quaternionf q;
      q = Eigen::AngleAxisf(state.orientation_[0], Eigen::Vector3f::UnitX())
        * Eigen::AngleAxisf(state.orientation_[1], Eigen::Vector3f::UnitY())
        * Eigen::AngleAxisf(state.orientation_[2], Eigen::Vector3f::UnitZ());
      
      odom_msg.pose.pose.orientation.x = q.x();
      odom_msg.pose.pose.orientation.y = q.y();
      odom_msg.pose.pose.orientation.z = q.z();
      odom_msg.pose.pose.orientation.w = q.w();

      return odom_msg;
    }

    Eigen::Matrix4f get_matrix_from_tf(const geometry_msgs::msg::TransformStamped tf)
    {
      Eigen::Matrix4f transform;
      transform(0, 3) = tf.transform.translation.x;
      transform(1, 3) = tf.transform.translation.y;
      transform(2, 3) = tf.transform.translation.z;
      transform(3, 3) = 1.0;
  
      Eigen::Quaternionf q(tf.transform.rotation.w, tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z);
      Eigen::Matrix3f rotation = q.toRotationMatrix();
  
      // Adjust the rotation matrix to account for the frame transformation
      Eigen::Matrix3f camera_to_base;
      camera_to_base << 1, 0, 0,
                        0, 1, 0,
                        0, 0, 1;
  
      rotation = camera_to_base * rotation;
      transform.block<3, 3>(0, 0) = rotation;
      return transform;
    }

    NavFilter filter_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr kalman_filter_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr mavros_subscription_;

    sensor_msgs::msg::Imu initial_imu_;
    geometry_msgs::msg::PoseStamped initial_pose_;
    geometry_msgs::msg::TwistStamped initial_twist_;

    bool imu_initialized_;
    bool pose_initialized_;
    bool twist_initialized_;

    std::string base_link_;
    std::string twist_link_;
    std::string pose_link_;
    std::string imu_link_;

    geometry_msgs::msg::TransformStamped imu_transform_;
    geometry_msgs::msg::TransformStamped twist_transform_;
    geometry_msgs::msg::TransformStamped pose_transform_;

    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto filter_node = std::make_shared<NavFilterNode>();
  rclcpp::spin(filter_node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}