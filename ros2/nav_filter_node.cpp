#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"


using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class NavFilter : public rclcpp::Node
{
  public:
    NavFilter()
    : Node("nav_filter"), count_(0)
    {
      publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

      imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu", 10, std::bind(&NavFilter::imu_callback, this, std::placeholders::_1));

      pose_stamped_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/pose_stamped", 10, std::bind(&NavFilter::pose_callback, this, std::placeholders::_1));

      twist_stamped_subscription_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "/twist", 10, std::bind(&NavFilter::twist_callback, this, std::placeholders::_1));
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


    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_stamped_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_stamped_subscription_;

    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavFilter>());
  rclcpp::shutdown();
  return 0;
}