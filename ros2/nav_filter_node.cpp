#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;
using rclcpp_lifecycle::LifecycleNode;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class NavFilter : public LifecycleNode
{
  public:
    NavFilter(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : LifecycleNode("nav_filter", options),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_)
    {
      this->declare_parameter<std::string>("base_link", "base_link");
      this->declare_parameter<std::string>("imu_link",  "base_link");
      this->declare_parameter<std::string>("twist_link",  "base_link");
      this->declare_parameter<std::string>("pose_link",  "base_link");

      publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

      imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu", 10, std::bind(&NavFilter::imu_callback, this, std::placeholders::_1));

      pose_stamped_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/pose_stamped", 10, std::bind(&NavFilter::pose_callback, this, std::placeholders::_1));

      twist_stamped_subscription_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "/twist", 10, std::bind(&NavFilter::twist_callback, this, std::placeholders::_1));
    }
  protected:
    CallbackReturn on_configure(const rclcpp_lifecycle::State &)
    { 
      RCLCPP_INFO(get_logger(), "Configuring Navigation Filter");
      try
      {
        rclcpp::Time now = this->get_clock()->now();

        imu_transform_ = tf_buffer_.lookupTransform(
                "base_link", "imu_link", tf2::TimePointZero);
        twist_transform_ = tf_buffer_.lookupTransform(
                "imu_link", "twist_link", tf2::TimePointZero);
        pose_transform_ = tf_buffer_.lookupTransform(
                "imu_link", "pose_link", tf2::TimePointZero);
      }
      catch(const tf2::TransformException & ex)
      {
        RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
        return CallbackReturn::FAILURE;
      }

      return CallbackReturn::SUCCESS;
    }

  private:
    void imu_callback(const sensor_msgs::msg::Imu & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "IMU message received");
      auto message = nav_msgs::msg::Odometry();
      RCLCPP_INFO(this->get_logger(), "Publishing");
      publisher_->publish(message);
    }
    void pose_callback(const geometry_msgs::msg::PoseStamped & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "Pose message received");
    }
    void twist_callback(const geometry_msgs::msg::TwistStamped & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "Twist message received");
    }

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_stamped_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_stamped_subscription_;

    geometry_msgs::msg::TransformStamped imu_transform_;
    geometry_msgs::msg::TransformStamped twist_transform_;
    geometry_msgs::msg::TransformStamped pose_transform_;

    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto filter_node = std::make_shared<NavFilter>();
  rclcpp::spin(filter_node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}