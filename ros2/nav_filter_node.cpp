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
#include "lifecycle_msgs/msg/state.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "nav_filter.hpp"

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
      this->declare_parameter<std::string>("base_link", "base_link");
      this->declare_parameter<std::string>("imu_link",  "base_link");
      this->declare_parameter<std::string>("twist_link",  "base_link");
      this->declare_parameter<std::string>("pose_link",  "base_link");

      publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

      imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu", 10, std::bind(&NavFilterNode::imu_callback, this, std::placeholders::_1));

      pose_stamped_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/pose_stamped", 10, std::bind(&NavFilterNode::pose_callback, this, std::placeholders::_1));

      twist_stamped_subscription_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "/twist", 10, std::bind(&NavFilterNode::twist_callback, this, std::placeholders::_1));
    }
  protected:
    CallbackReturn on_configure(const rclcpp_lifecycle::State &)
    { 
      RCLCPP_INFO(get_logger(), "Configuring Navigation Filter");
      try
      {
        rclcpp::Time now = this->get_clock()->now();

        this->get_parameter("base_link", base_link_);
        this->get_parameter("imu_link", imu_link_);
        this->get_parameter("twist_link", twist_link_);
        this->get_parameter("pose_link", pose_link_);

        imu_transform_ = tf_buffer_.lookupTransform(
                base_link_, imu_link_, tf2::TimePointZero);
        twist_transform_ = tf_buffer_.lookupTransform(
                imu_link_, twist_link_, tf2::TimePointZero);
        pose_transform_ = tf_buffer_.lookupTransform(
                imu_link_, pose_link_, tf2::TimePointZero);

        RCLCPP_INFO(get_logger(), "Navigation Filter Configured");
      }
      catch(const tf2::TransformException & ex)
      {
        RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
        return CallbackReturn::FAILURE;
      }

      return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State &)
    {
      RCLCPP_INFO(this->get_logger(), "Activating Navigation Filter");
      
      return CallbackReturn::SUCCESS;
    }

  private:
    void imu_callback(const sensor_msgs::msg::Imu & msg)
    {
      if (this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
      {
        RCLCPP_INFO(this->get_logger(), "IMU message received");
        auto message = nav_msgs::msg::Odometry();
        RCLCPP_INFO(this->get_logger(), "Publishing");
        publisher_->publish(message);
      }
    }
    void pose_callback(const geometry_msgs::msg::PoseStamped & msg)
    {
      if (this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
      {
        RCLCPP_INFO(this->get_logger(), "Pose message received");
      }
    }
    void twist_callback(const geometry_msgs::msg::TwistStamped & msg)
    {
      if (this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
      {
        RCLCPP_INFO(this->get_logger(), "Twist message received");
      }
    }

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_stamped_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_stamped_subscription_;

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