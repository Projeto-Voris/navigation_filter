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
    this->declare_parameter<double>("dvl_variance_threshold", 1.0);
    this->get_parameter("dvl_variance_threshold", dvl_variance_threshold_);

    // Inicialização segura das matrizes de transformação geométrica
    T_offset_slam_ = Eigen::Isometry3d::Identity();
    T_offset_dvl_ = Eigen::Isometry3d::Identity();
    T_last_fused_ = Eigen::Isometry3d::Identity();
    T_last_dvl_ = Eigen::Isometry3d::Identity();
    T_current_slam = Eigen::Isometry3d::Identity();
    
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

    auto cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    reset_dvl_pose = this->create_client<std_srvs::srv::Trigger>("/waterlinked_dvl_driver/reset_dead_reckoning", rmw_qos_profile_default, cb_group);
}

void PoseFusionComponent::slamPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    tf2::fromMsg(msg->pose.pose, T_current_slam);
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
    this->get_parameter("dvl_variance_threshold", dvl_variance_threshold_);
    double alpha_target = 0.7; 
    double current_alpha = 0.0; // Começa em zero até o SLAM estabilizar

    // Tratamento do primeiro ciclo do DVL para ancoragem do delta relativo
    if (!is_dvl_initialized_) {
        T_last_dvl_ = T_current_dvl;
        is_dvl_initialized_ = true;
        return;
    }

    // 1. Verificação da saúde operacional dos sensores
    bool slam_ok = (this->current_tracking_state_ == 2 || this->current_tracking_state_ == 5);
    bool dvl_ok = (msg->pose.covariance[0] < dvl_variance_threshold_);

    if(!dvl_ok){

        RCLCPP_WARN(this->get_logger(), "DVL Covariance above threshold %f", dvl_variance_threshold_);
        T_offset_dvl_ = T_last_fused_;
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

        using ServiceResponseFuture = rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture;
        auto response_callback = [this](ServiceResponseFuture future) {
        auto result = future.get();
        if (result->success) {
            RCLCPP_INFO(this->get_logger(), "Reseted: %s", result->message.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "Fail Reset: %s", result->message.c_str());
        }
        };
        reset_dvl_pose->async_send_request(request, response_callback);

    }

    // Detecção de eventos de transição ou reinicializações de mapas do SLAM
    bool map_has_changed = map_changed_ || (current_map_id_ != last_map_id_);
    bool slam_recovered = slam_ok && (last_tracking_state_ != 2 && last_tracking_state_ != 5);

    Eigen::Isometry3d T_out_slam = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d T_out_dvl = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d T_fused = Eigen::Isometry3d::Identity();

    std::array<double, 36> fused_covariance;
    // 2. Execução da Máquina de Estados de Fusão Cinemática
    if (slam_ok && has_slam_pose_) {
        // --- MODO 1: SLAM SAUDÁVEL (Referência Absoluta Ativa) ---
        if (!is_offset_initialized_ || map_has_changed || slam_recovered) {
            // Recálculo imediato do offset matemático para neutralizar saltos na malha do EKF
            T_offset_slam_ = T_last_fused_; 
            is_offset_initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "Discontinuidade do SLAM absorvida. Novo T_offset calculado.");
        }
        if (current_alpha < alpha_target) {
        current_alpha += 0.05; // Ajuste este passo para controlar a velocidade da transição
        if (current_alpha > alpha_target) current_alpha = alpha_target;
    }

        T_out_slam = T_offset_slam_ * T_current_slam; 
    }
    else{
        current_alpha = 0.0;
    }
     if (dvl_ok) {

        T_out_dvl = T_offset_dvl_ * T_current_dvl; 

        // Propaga e incrementa continuamente a incerteza no tempo
        fused_covariance = msg->pose.covariance; // Para simplificação, utilizando a covariância atual do DVL. Ajuste conforme necessário.
    }
    
    if(dvl_ok && slam_ok){
            // 2. Extração de Translação e Rotação das duas poses já corrigidas pelos offsets
        Eigen::Vector3d pos_slam = T_out_slam.translation();
        Eigen::Vector3d pos_dvl  = T_out_dvl.translation();

        Eigen::Quaterniond rot_slam(T_out_slam.rotation());
        Eigen::Quaterniond rot_dvl(T_out_dvl.rotation());

        // 3. Aplicação do Filtro Complementar (LERP + SLERP)
        Eigen::Vector3d pos_fused = current_alpha * pos_slam + (1.0 - current_alpha) * pos_dvl;
        Eigen::Quaterniond rot_fused = rot_dvl.slerp(current_alpha, rot_slam); // Rotaciona de DVL em direção a SLAM por 'alpha'

        T_fused.translation() = pos_fused;
        T_fused.linear() = rot_fused.toRotationMatrix();
    }
    else if(dvl_ok && !slam_ok){

        T_fused = T_out_dvl;

    } 
    else{
        // --- MODO 3: FALHA CRÍTICA GERAL (ROV Completamente Cego) ---
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Ambos os sensores falharam! Travando posição.");
        T_fused = T_last_fused_; // Estabiliza na última pose válida para mitigar acelerações espúrias
        
        fused_covariance.fill(0.0);
        fused_covariance[0] = -1.0;  // X
        fused_covariance[7] = -1.0;  // Y
        fused_covariance[14] = -1.0; // Z
    }


    // 3. Empacotamento em unique_ptr (Indispensável para Zero-Copy via IPC)
    auto msg_out = std::make_unique<geometry_msgs::msg::PoseWithCovarianceStamped>();
    msg_out->header.stamp = now();
    msg_out->header.frame_id = "map";
    msg_out->pose.pose = tf2::toMsg(T_fused);
    msg_out->pose.covariance = fused_covariance;

    // 4. Atualização do Histórico do Sistema
    T_last_fused_ = T_fused;
    T_last_dvl_ = T_current_dvl;
    T_last_fused_covariance_ = fused_covariance;
    last_tracking_state_ = current_tracking_state_;
    last_map_id_ = current_map_id_;

    // Publicação limpa por transferência de propriedade (move semantics)
    fused_pose_pub_->publish(std::move(msg_out));
}

} // namespace nav_filter

// Macro ROS 2 para exportar e registrar o componente no gerenciador de nós dinâmicos
RCLCPP_COMPONENTS_REGISTER_NODE(nav_filter::PoseFusionComponent)