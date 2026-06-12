#ifndef VORIS_FUSION__POSE_FUSION_COMPONENT_HPP_
#define VORIS_FUSION__POSE_FUSION_COMPONENT_HPP_

#include <memory>
#include <array>
#include <vector>
#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <orbslam3_msgs/msg/slam_status.hpp>
#include <std_srvs/srv/trigger.hpp>

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
    // --- Callbacks dos Assinantes ---
    void slamPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void dvlPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    
    // Callback temporário/mock para o status do SLAM usando uma mensagem padrão.
    // Altere para o tipo correto da sua interface assim que integrá-la.
    void slamStatusCallback(const orbslam3_msgs::msg::SlamStatus::SharedPtr msg);

    // --- Funções Auxiliares Matemáticas ---
    std::array<double, 36> accumulateCovariance(
        const std::array<double, 36>& current_fused, 
        const std::array<double, 36>& dvl_cov, 
        double dt);

    // --- Objetos de Comunicação do ROS 2 ---
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr slam_pose_sub_;
    rclcpp::Subscription<orbslam3_msgs::msg::SlamStatus>::SharedPtr slam_status_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr dvl_pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr fused_pose_pub_;

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_dvl_pose;

    // --- Variáveis de Estado (Matrizes Homogêneas) ---
    Eigen::Isometry3d T_offset_slam_;
    Eigen::Isometry3d T_offset_dvl_;
    Eigen::Isometry3d T_last_fused_;
    Eigen::Isometry3d T_last_dvl_;
    Eigen::Isometry3d T_current_slam;

    // --- Matrizes de Covariância ---
    std::array<double, 36> T_last_fused_covariance_;
    std::array<double, 36> slam_covariance_;

    // --- Máquina de Estados Discretos ---
    int8_t current_tracking_state_{-1};
    int8_t last_tracking_state_{-1};
    uint32_t current_map_id_{0};
    uint32_t last_map_id_{0};
    bool map_changed_{false};

    // --- Flags de Controle de Inicialização ---
    bool is_dvl_initialized_{false};
    bool is_offset_initialized_{false};
    bool has_slam_pose_{false};
    
    // --- Parâmetros e Tempo ---
    double dvl_variance_threshold_;
    rclcpp::Time last_stamp_;
};

} // namespace nav_filter

#endif // VORIS_FUSION__POSE_FUSION_COMPONENT_HPP_