#ifndef VORIS_FUSION__POSE_FUSION_COMPONENT_HPP_
#define VORIS_FUSION__POSE_FUSION_COMPONENT_HPP_

#include "rclcpp/rclcpp.hpp"
#include <tf2_eigen/tf2_eigen.hpp> 
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <orbslam3_msgs/msg/slam_status.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <mavros_msgs/srv/command_home.hpp>
#include <mavros_msgs/msg/vfr_hud.hpp>

#include <memory>
#include <array>
#include <vector>
#include <Eigen/Geometry>


// NOTA: Se você tiver um pacote de mensagens gerado para o status do SLAM, 
// inclua o header real aqui e substitua o tipo nos tópicos.
// #include "voris_interfaces/msg/slam_status.hpp"

namespace nav_filter
{

class PoseFusionComponent : public rclcpp::Node
{
public:
    explicit PoseFusionComponent(const rclcpp::NodeOptions & options);
    virtual ~PoseFusionComponent() = default;

private:
    void slamPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void dvlPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void slamStatusCallback(const orbslam3_msgs::msg::SlamStatus::SharedPtr msg);
    void vfrHudCallback(const mavros_msgs::msg::VfrHud::SharedPtr msg);

    void send_fake_home();
    void print_tf(Eigen::Isometry3d tf);

    // Poses, VRF_HUD, slam status subscribers
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr slam_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr dvl_pose_sub_;
    rclcpp::Subscription<orbslam3_msgs::msg::SlamStatus>::SharedPtr slam_status_sub_;
    rclcpp::Subscription<mavros_msgs::msg::VfrHud>::SharedPtr vfr_hud_sub_;

    // Fused pose publisher
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr fused_pose_pub_;
    // Debug publishers to visualize offsets
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr slam_debug_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr dvl_debug_pub_;


    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_dvl_pose;
    rclcpp::Client<mavros_msgs::srv::CommandHome>::SharedPtr set_home_client_;
    
    rclcpp::TimerBase::SharedPtr home_trigger_timer_;

    // Transform matrixes
    Eigen::Isometry3d T_offset_slam_;
    Eigen::Isometry3d T_offset_dvl_;
    Eigen::Isometry3d T_slam_yaw_offset_;
    Eigen::Isometry3d T_dvl_yaw_offset_;
    Eigen::Isometry3d T_last_fused_;
    Eigen::Isometry3d T_last_dvl_;
    Eigen::Isometry3d T_current_slam;
    Eigen::Isometry3d T_current_dvl;

    // Covariance variables
    std::array<double, 36> T_last_fused_covariance_;
    std::array<double, 36> slam_covariance_;

    // State variables
    int8_t current_tracking_state_{-1};
    int8_t last_tracking_state_{-1};
    uint32_t current_map_id_{0};
    uint32_t last_map_id_{0};
    bool map_changed_{false};

    // Flags
    bool is_dvl_initialized_{false};
    bool is_offset_initialized_{false};
    bool has_slam_pose_{false};
    bool is_slam_yaw_alighned_{false};
    bool is_dvl_yaw_alighned_{false};
    
    // time parametrs
    double dvl_variance_threshold_;
    rclcpp::Time last_stamp_;
};

} // namespace nav_filter

#endif // VORIS_FUSION__POSE_FUSION_COMPONENT_HPP_