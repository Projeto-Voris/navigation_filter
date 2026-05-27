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
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
// #include "dvl_msgs/msg/dvl.hpp"
#include "nav_filter.hpp"
#include "imu_model.hpp"

using namespace std::chrono_literals;
using rclcpp_lifecycle::LifecycleNode;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */


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
      this->declare_parameter<std::string>("imu.link",  "base_link");
      this->declare_parameter<std::vector<double>>("imu.accelerometer_random_walk_bias", {0,0,0});
      this->declare_parameter<std::vector<double>>("imu.gyroscope_random_walk_bias", {0,0,0});
      this->declare_parameter<std::vector<double>>("imu.accelerometer_noise", {0,0,0});
      this->declare_parameter<std::vector<double>>("imu.gyroscope_noise", {0,0,0});
      this->declare_parameter<std::vector<double>>("imu.correlation_noise", {0,0,0});
      this->declare_parameter<std::vector<double>>("imu.correlation_matrix.row0", {0,0,0});
      this->declare_parameter<std::vector<double>>("imu.correlation_matrix.row1", {0,0,0});
      this->declare_parameter<std::vector<double>>("imu.correlation_matrix.row2", {0,0,0});
      this->declare_parameter<float>("imu.update_time",  0.01);


      this->declare_parameter<std::string>("twist.link",  "base_link");
      this->declare_parameter<std::vector<float>>("twist.noise", {0,0,0});
      this->declare_parameter<float>("twist.update_time",  0.01);

      this->declare_parameter<std::string>("pose.link",  "base_link");
      this->declare_parameter<std::vector<float>>("pose.noise", {0,0,0});
      this->declare_parameter<float>("pose.update_time",  0.01);

      /* Publish a Odometry msg */

      publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/nav_filter/odom", 10);
      
      /* Receives a imu, pose and twist stamped */

      // IMU IMU.msgs Input ---> angular orientations, velocities and linear accelerations
      imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/mavros/imu/data_raw", rclcpp::SensorDataQoS(), std::bind(&NavFilterNode::imu_callback, this, std::placeholders::_1));
      
      // SLAM PoseStamped Input ---> position and orientation       
      pose_stamped_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mavros/local_position/pose", 10, std::bind(&NavFilterNode::pose_callback, this, std::placeholders::_1));
      
      // DVL DVL.msgs Input ---> linear velocities
      twist_stamped_subscription_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
      "/dvl_twist_with_covariance", 10, std::bind(&NavFilterNode::dvl_callback, this, std::placeholders::_1));
    
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

        this->get_parameter("imu.link", imu_link_);
        std::vector<double> acc_bias = this->get_parameter("imu.accelerometer_random_walk_bias").as_double_array();
        std::vector<double> gyro_bias = this->get_parameter("imu.gyroscope_random_walk_bias").as_double_array();
        std::vector<double> acc_noise = this->get_parameter("imu.accelerometer_noise").as_double_array();
        std::vector<double> gyro_noise = this->get_parameter("imu.gyroscope_noise").as_double_array();
        std::vector<double> corr_noise = this->get_parameter("imu.correlation_noise").as_double_array();
        std::vector<double> corr_row_0 = this->get_parameter("imu.correlation_matrix.row0").as_double_array();
        std::vector<double> corr_row_1 = this->get_parameter("imu.correlation_matrix.row1").as_double_array();
        std::vector<double> corr_row_2 = this->get_parameter("imu.correlation_matrix.row2").as_double_array();
        float imu_update_time = static_cast<float>(this->get_parameter("imu.update_time").as_double());

        this->get_parameter("twist.link", twist_link_);
        std::vector<double> twist_noise = this->get_parameter("twist.noise").as_double_array();
        float twist_update_time = static_cast<float>(this->get_parameter("twist.update_time").as_double());

        this->get_parameter("pose.link", pose_link_);
        std::vector<double> pose_noise = this->get_parameter("pose.noise").as_double_array();
        float pose_update_time = static_cast<float>(this->get_parameter("pose.update_time").as_double());

        imu_transform_ = tf_buffer_.lookupTransform(
                base_link_, imu_link_, tf2::TimePointZero, tf2::durationFromSec(1.0));
        twist_transform_ = tf_buffer_.lookupTransform(
                imu_link_, twist_link_, tf2::TimePointZero, tf2::durationFromSec(1.0));
        pose_transform_ = tf_buffer_.lookupTransform(
                imu_link_, pose_link_, tf2::TimePointZero, tf2::durationFromSec(1.0));

        Eigen::Matrix4f imu_transform_matrix = NavFilterNode::get_matrix_from_tf(imu_transform_);
        Eigen::Matrix4f twist_transform_matrix = NavFilterNode::get_matrix_from_tf(twist_transform_);
        Eigen::Matrix4f pose_transform_matrix = NavFilterNode::get_matrix_from_tf(pose_transform_);

        Eigen::Matrix3d corr_matrix (3,3);

        corr_matrix << corr_row_0[0], corr_row_0[1], corr_row_0[2],
                      corr_row_1[0], corr_row_1[1], corr_row_1[2],
                      corr_row_2[0], corr_row_2[1], corr_row_2[2];

        IMUModel imu_model(imu_transform_matrix,acc_bias, gyro_bias, acc_noise, gyro_noise, corr_noise, corr_matrix, float(imu_update_time));
        TwistModel twist_model(twist_transform_matrix, twist_noise, float(twist_update_time));
        PoseModel pose_model(pose_transform_matrix, pose_noise, float(pose_update_time));

        NavFilter filter_(imu_model, twist_model, pose_model);

        RCLCPP_INFO(get_logger(), "Base: %s", base_link_.c_str());
        RCLCPP_INFO(get_logger(), "IMU: %s", imu_link_.c_str());
        RCLCPP_INFO(get_logger(), "Twist: %s", twist_link_.c_str());
        RCLCPP_INFO(get_logger(), "Pose: %s", pose_link_.c_str());

        RCLCPP_INFO(get_logger(), "Navigation Filter Configured");
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
      RCLCPP_INFO(this->get_logger(), "Activating Navigation Filter");
      rclcpp::Rate rate(0.5); // 0.5 Hz (2 seconds delay)
      rate.sleep();

      RCLCPP_INFO(this->get_logger(), "Navigation Filter Activated");
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

    void pose_callback(const geometry_msgs::msg::PoseStamped & msg)
    {
      if (this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
      {
        Eigen::Quaternionf q(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z);
        Eigen::Vector3f euler_angles = q.toRotationMatrix().eulerAngles(0, 1, 2);

        filter_.update_pose(Eigen::Matrix<float, 6, 1>(
          msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
          euler_angles[0], euler_angles[1], euler_angles[2]));
      }
    }


    // Dentro do dvl callback na real as únicas informações que realmente vao ser usadas são as velocidades lineares
    // No twist model, as velocidades angulares são todas zeradas
    void dvl_callback(const geometry_msgs::msg::TwistWithCovarianceStamped & msg)
    {
      if (this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
      {
        // Update the filter with the DVL linear velocity and IMU angular velocity measurements
        filter_.update_twist(Eigen::Matrix<float, 6, 1>(
          msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z,
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
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_stamped_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_stamped_subscription_;

    sensor_msgs::msg::Imu initial_imu_;
    geometry_msgs::msg::PoseStamped initial_pose_;
    geometry_msgs::msg::TwistWithCovarianceStamped initial_twist_;

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