#include "pose_fusion.hpp"

#include "rclcpp_components/register_node_macro.hpp"

namespace nav_filter
{

PoseFusionComponent::PoseFusionComponent(const rclcpp::NodeOptions & options)
: Node("pose_fusion_node", options)
{
    RCLCPP_INFO(this->get_logger(), "Inicializando Componente de Fusão de Pose...");

    // Declaração de parâmetros dinâmicos
    this->declare_parameter<double>("home_lat", -27.5951);
    this->declare_parameter<double>("home_long", -48.5637);
    this->declare_parameter<double>("home_alt", 0.0);
    this->declare_parameter<double>("dvl_variance_threshold", 1.0);
    this->get_parameter("dvl_variance_threshold", dvl_variance_threshold_);

    // Inicialização segura das matrizes de transformação geométrica
    T_offset_slam_ = Eigen::Isometry3d::Identity();
    T_offset_dvl_ = Eigen::Isometry3d::Identity();
    T_slam_yaw_offset_ = Eigen::Isometry3d::Identity();
    T_dvl_yaw_offset_ = Eigen::Isometry3d::Identity();
    T_last_fused_ = Eigen::Isometry3d::Identity();
    T_last_dvl_ = Eigen::Isometry3d::Identity();
    T_current_slam = Eigen::Isometry3d::Identity();
    T_current_dvl = Eigen::Isometry3d::Identity();

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
    
    vfr_hud_sub_ = this->create_subscription<mavros_msgs::msg::VfrHud>(
        "/mavros/vfr_hud", 10,
        std::bind(&PoseFusionComponent::vfrHudCallback, this, std::placeholders::_1));

    // --- Configuração do Publicador (Publisher) ---
    fused_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "fused/pose_cov", 10);
    slam_debug_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "slam/debug", 10);
    dvl_debug_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "dvl/debug", 10);


    auto cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto cb_home_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    
    set_home_client_ = this->create_client<mavros_msgs::srv::CommandHome>("/mavros/cmd/set_home", rmw_qos_profile_default, cb_home_group);
    reset_dvl_pose = this->create_client<std_srvs::srv::Trigger>("/waterlinked_dvl_driver/reset_dead_reckoning", rmw_qos_profile_default, cb_group);

    home_trigger_timer_ = this->create_wall_timer(std::chrono::seconds(5), [this](){
        this->send_fake_home();
        this->home_trigger_timer_->cancel();
    });
}


void PoseFusionComponent::send_fake_home() {
    if (!set_home_client_->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_WARN(this->get_logger(), "MAVROS set_home service not available yet.");
        return;
    }

    double lat = this->get_parameter("home_lat").as_double();
    double lon = this->get_parameter("home_lon").as_double();
    double alt = this->get_parameter("home_alt").as_double();

    auto request = std::make_shared<mavros_msgs::srv::CommandHome::Request>();
    request->current_gps = false;
    request->latitude = lat;
    request->longitude = lon;
    request->altitude = alt;

    RCLCPP_INFO(this->get_logger(), "Anchoring EKF Origin to Tank at Lat: %f, Lon: %f", lat, lon);

    auto result_future = set_home_client_->async_send_request(
        request,
        [this](rclcpp::Client<mavros_msgs::srv::CommandHome>::SharedFuture future) {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "Home set successfully! ArduSub barometer unlocked.");
            } else {
                RCLCPP_ERROR(this->get_logger(), "ArduSub rejected Home initialization.");
            }
        }
    );
    auto request_dvl = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto response_callback = [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
    auto result = future.get();
    if (result->success) {
        RCLCPP_INFO(this->get_logger(), "Reseted: %s", result->message.c_str());
    } else {
        RCLCPP_WARN(this->get_logger(), "Fail Reset: %s", result->message.c_str());
    }
    };
    reset_dvl_pose->async_send_request(request_dvl, response_callback);

}

void PoseFusionComponent::vfrHudCallback(const mavros_msgs::msg::VfrHud::SharedPtr msg) {
    // Aqui você pode processar os dados do VFR_HUD conforme necessário
    RCLCPP_INFO(this->get_logger(), "VFR_HUD received: Heading: %d, Altitude: %f", msg->heading, msg->altitude);
    double heading_rad = static_cast<double>(msg->heading) * M_PI / 180.0; // Convert to radians
    double yaw_compass_enu = (M_PI / 2.0) - heading_rad; // Convert compass heading to ENU yaw
    yaw_compass_enu = atan2(sin(yaw_compass_enu), cos(yaw_compass_enu)); // Normalize to [-pi, pi]
    RCLCPP_INFO(this->get_logger(), "Converted ENU Yaw: %f rad", yaw_compass_enu);

    if(!is_slam_yaw_alighned_ && has_slam_pose_){
        Eigen::Vector3d euler_angles = T_current_slam.rotation().eulerAngles(2, 1, 0);
        double slam_yaw = euler_angles[0]; // Yaw angle from SLAM
        double yaw_offset = yaw_compass_enu - slam_yaw;
        T_slam_yaw_offset_.rotate(Eigen::AngleAxisd(yaw_offset, Eigen::Vector3d::UnitZ()));
        is_slam_yaw_alighned_ = true;
        RCLCPP_INFO(this->get_logger(), "SLAM Yaw aligned with Compass. Offset: %f rad", yaw_offset);
        print_tf(T_slam_yaw_offset_);
    }
    if(!is_dvl_yaw_alighned_ && is_dvl_initialized_){
        Eigen::Vector3d euler_angles_dvl = T_current_dvl.rotation().eulerAngles(2, 1, 0);
        double dvl_yaw = euler_angles_dvl[0]; // Yaw angle from DVL
        double yaw_offset_dvl = yaw_compass_enu - dvl_yaw;
        T_dvl_yaw_offset_.rotate(Eigen::AngleAxisd(yaw_offset_dvl, Eigen::Vector3d::UnitZ()));
        is_dvl_yaw_alighned_ = true;
        RCLCPP_INFO(this->get_logger(), "DVL Yaw aligned with Compass. Offset: %f rad", yaw_offset_dvl);
        print_tf(T_dvl_yaw_offset_);
    }
}

void PoseFusionComponent::slamPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    tf2::fromMsg(msg->pose.pose, T_current_slam);
    has_slam_pose_ = true;
}

void PoseFusionComponent::slamStatusCallback(const orbslam3_msgs::msg::SlamStatus::SharedPtr msg)
{
    this->current_tracking_state_ = msg->tracking_state;
    this->current_map_id_ = msg->map_id;
    this->map_changed_ = msg->map_changed;
}

void PoseFusionComponent::dvlPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    
    tf2::fromMsg(msg->pose.pose, T_current_dvl);
    this->get_parameter("dvl_variance_threshold", dvl_variance_threshold_);
    double alpha_target = 0.7; 
    double current_alpha = 0.0; // Começa em zero até o SLAM estabilizar

    if (!is_dvl_yaw_alighned_ && !is_dvl_initialized_) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting VFR_HUD topic");
    }

    Eigen::Isometry3d T_slam_aligned = T_slam_yaw_offset_ * T_current_slam;
    Eigen::Isometry3d T_dvl_aligned  = T_dvl_yaw_offset_ * T_current_dvl;

    if (!is_dvl_initialized_) {
        T_last_dvl_ = T_dvl_aligned;
        is_dvl_initialized_ = true;
        return;
    }

    // 1. Verificação da saúde operacional dos sensores
    bool slam_ok = (this->current_tracking_state_ == 2 || this->current_tracking_state_ == 5);
    bool dvl_ok = (msg->pose.covariance[0] < dvl_variance_threshold_);

    if(!dvl_ok){

        RCLCPP_WARN(this->get_logger(), "DVL Covariance above threshold %f", dvl_variance_threshold_);
        T_offset_dvl_ = T_last_fused_;
        print_tf(T_offset_dvl_);
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto response_callback = [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
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
        // 
        if (!is_offset_initialized_ || map_has_changed || slam_recovered) {
            // If slam change coordinates, restabilish new offset
            T_offset_slam_ = T_last_fused_;
            print_tf(T_offset_slam_); 
            is_offset_initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "Discontinuidade do SLAM absorvida. Novo T_offset calculado.");
        }
        if (current_alpha < alpha_target) {
        current_alpha += 0.05; // Ajuste este passo para controlar a velocidade da transição
        if (current_alpha > alpha_target) current_alpha = alpha_target;
    }

        T_out_slam = T_offset_slam_ * T_slam_aligned; 
    }
    else{
        current_alpha = 0.0;
    }
     if (dvl_ok) {

        T_out_dvl = T_offset_dvl_ * T_dvl_aligned; 

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
    T_last_dvl_ = T_dvl_aligned;
    T_last_fused_covariance_ = fused_covariance;
    last_tracking_state_ = current_tracking_state_;
    last_map_id_ = current_map_id_;

    // Publicação limpa por transferência de propriedade (move semantics)
    fused_pose_pub_->publish(std::move(msg_out));

    msg_out->pose.pose = tf2::toMsg(T_out_slam);
    slam_debug_pub_->publish(std::move(msg_out));

    msg_out->pose.pose = tf2::toMsg(T_out_dvl);
    dvl_debug_pub_->publish(std::move(msg_out));
}

void PoseFusionComponent::print_tf(Eigen::Isometry3d tf){

    Eigen::Matrix3d rotation = tf.rotation();
    Eigen::Vector3d euler_angles = rotation.eulerAngles(2,1,0);

    double roll = euler_angles[2]* (180.0 / M_PI);
    double pitch = euler_angles[1]* (180.0 / M_PI);
    double yaw = euler_angles[0]* (180.0 / M_PI);

    RCLCPP_INFO(this->get_logger(), "Translation: [%f, %f, %f]", tf.translation().x(), tf.translation().y(), tf.translation().z());
    RCLCPP_INFO(this->get_logger(), "RPY angles: [%f, %f, %f] deg", roll, pitch, yaw);
}
} // namespace nav_filter

// Macro ROS 2 para exportar e registrar o componente no gerenciador de nós dinâmicos
RCLCPP_COMPONENTS_REGISTER_NODE(nav_filter::PoseFusionComponent)