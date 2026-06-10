#include "pose_fusion.hpp"

#include <tf2_eigen/tf2_eigen.hpp> // Essencial para as conversões entre ROS Msg e Eigen
#include "rclcpp_components/register_node_macro.hpp"

namespace nav_filter
{

PoseFusionComponent::PoseFusionComponent(const rclcpp::NodeOptions & options)
: Node("pose_fusion_node", options)
{
    RCLCPP_INFO(this->get_logger(), "Inicializando Componente de Fusão de Pose...");

    // Declaração de parâmetros dinâmicos
    this->declare_parameter<double>("dvl_variance_threshold", 10.0);
    this->get_parameter("dvl_variance_threshold", dvl_variance_threshold_);

    // Inicialização segura das matrizes de transformação geométrica
    T_offset_ = Eigen::Isometry3d::Identity();
    T_last_fused_ = Eigen::Isometry3d::Identity();
    T_last_dvl_ = Eigen::Isometry3d::Identity();
    T_last_slam_ = Eigen::Isometry3d::Identity();
    
    T_last_fused_covariance_.fill(0.0);

    // --- Configuração dos Assinantes (Subscribers) ---
    // Inscrição assíncrona do SLAM para evitar gargalos na taxa principal do DVL
    slam_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "slam/pose_cov", 10,
        std::bind(&PoseFusionComponent::slamPoseCallback, this, std::placeholders::_1));

    slam_status_sub_ = this->create_subscription<orbslam3_msgs::msg::SlamStatus>(
        "slam/status", 10,
        std::bind(&PoseFusionComponent::slamStatusCallback, this, std::placeholders::_1));

    // O DVL dita a frequência de execução e atualização da malha do MAVROS
    dvl_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "dvl/pose_cov", 10,
        std::bind(&PoseFusionComponent::dvlPoseCallback, this, std::placeholders::_1));

    // --- Configuração do Publicador (Publisher) ---
    fused_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "fused/pose_cov", 10);
    
    last_stamp_ = this->now();
}

void PoseFusionComponent::slamPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    tf2::fromMsg(msg->pose.pose, T_last_slam_);
    slam_covariance_ = msg->pose.covariance;
    has_slam_pose_ = true;
}

void PoseFusionComponent::slamStatusCallback(const orbslam3_msgs::msg::SlamStatus::SharedPtr msg)
{
    // Mock local dos estados informados. 
    // Quando utilizar a sua mensagem customizada, faça a atribuição direta dos campos equivalentes.
    this->current_tracking_state_ = msg->tracking_state; // Exemplo: Forçando TRACKING_OK (2) ou TRACKING_OK_KLT (5)
    this->current_map_id_ = msg->map_id;
    this->map_changed_ = msg->map_changed;
}

void PoseFusionComponent::dvlPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    Eigen::Isometry3d T_current_dvl;
    tf2::fromMsg(msg->pose.pose, T_current_dvl);

    // Tratamento do primeiro ciclo do DVL para ancoragem do delta relativo
    if (!is_dvl_initialized_) {
        T_last_dvl_ = T_current_dvl;
        is_dvl_initialized_ = true;
        return;
    }

    // 1. Verificação da saúde operacional dos sensores
    bool slam_ok = (current_tracking_state_ == 2 || current_tracking_state_ == 5);
    bool dvl_ok = (msg->pose.covariance[0] < dvl_variance_threshold_);

    // Cálculo do Delta de Tempo entre iterações sucessivas
    rclcpp::Time current_stamp = msg->header.stamp;
    double dt = (current_stamp - last_stamp_).seconds();
    if (dt <= 0.0 || dt > 1.0) dt = 0.05;

    // Detecção de eventos de transição ou reinicializações de mapas do SLAM
    bool map_has_changed = map_changed_ || (current_map_id_ != last_map_id_);
    bool slam_recovered = slam_ok && (last_tracking_state_ != 2 && last_tracking_state_ != 5);

    Eigen::Isometry3d T_out = Eigen::Isometry3d::Identity();
    std::array<double, 36> fused_covariance;

    // 2. Execução da Máquina de Estados de Fusão Cinemática
    if (slam_ok && has_slam_pose_) {
        // --- MODO 1: SLAM SAUDÁVEL (Referência Absoluta Ativa) ---
        if (!is_offset_initialized_ || map_has_changed || slam_recovered) {
            // Recálculo imediato do offset matemático para neutralizar saltos na malha do EKF
            T_offset_ = T_last_fused_ * T_last_slam_.inverse(); 
            is_offset_initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "Discontinuidade do SLAM absorvida. Novo T_offset calculado.");
        }

        T_out = T_offset_ * T_last_slam_; 
        fused_covariance = slam_covariance_; // Preserva a covariância estática e confiável do SLAM [cite: 119]
    }
    else if (dvl_ok) {
        // --- MODO 2: DVL FALLBACK (Navegação Estimada por Odometria Relativa) ---
        Eigen::Isometry3d T_dvl_delta = T_last_dvl_.inverse() * T_current_dvl;
        T_out = T_last_fused_ * T_dvl_delta; 

        // Propaga e incrementa continuamente a incerteza no tempo
        fused_covariance = accumulateCovariance(T_last_fused_covariance_, msg->pose.covariance, dt);
    }
    else {
        // --- MODO 3: FALHA CRÍTICA GERAL (ROV Completamente Cego) ---
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Ambos os sensores falharam! Travando posição.");
        T_out = T_last_fused_; // Estabiliza na última pose válida para mitigar acelerações espúrias
        
        fused_covariance.fill(0.0);
        fused_covariance[0] = 50.0;  // X
        fused_covariance[7] = 50.0;  // Y
        fused_covariance[14] = 50.0; // Z
        fused_covariance[35] = 5.0;  // Yaw [cite: 124]
    }

    // 3. Empacotamento em unique_ptr (Indispensável para Zero-Copy via IPC)
    auto msg_out = std::make_unique<geometry_msgs::msg::PoseWithCovarianceStamped>();
    msg_out->header.stamp = current_stamp;
    msg_out->header.frame_id = "odom_fused";
    msg_out->pose.pose = tf2::toMsg(T_out);
    msg_out->pose.covariance = fused_covariance;

    // 4. Atualização do Histórico do Sistema
    T_last_fused_ = T_out;
    T_last_dvl_ = T_current_dvl;
    T_last_fused_covariance_ = fused_covariance;
    last_tracking_state_ = current_tracking_state_;
    last_map_id_ = current_map_id_;
    last_stamp_ = current_stamp;

    // Publicação limpa por transferência de propriedade (move semantics)
    fused_pose_pub_->publish(std::move(msg_out));
}

std::array<double, 36> PoseFusionComponent::accumulateCovariance(
    const std::array<double, 36>& current_fused, 
    const std::array<double, 36>& dvl_cov, 
    double dt)
{
    std::array<double, 36> updated_cov = current_fused;
    const std::vector<int> diagonals = {0, 7, 14, 21, 28, 35}; // Mapeamento da diagonal principal 6x6
    
    double growth_factor = 1.5; // Ajuste para acelerar o crescimento do desvio no fallback [cite: 123]

    for (int idx : diagonals) {
        updated_cov[idx] += dvl_cov[idx] * dt * growth_factor; 
        
        // Teto de saturação seguro para evitar instabilidade numérica no filtro do piloto automático
        if (updated_cov[idx] > 40.0) {
            updated_cov[idx] = 40.0; 
        }
    }
    return updated_cov;
}

} // namespace nav_filter

// Macro ROS 2 para exportar e registrar o componente no gerenciador de nós dinâmicos
RCLCPP_COMPONENTS_REGISTER_NODE(nav_filter::PoseFusionComponent)