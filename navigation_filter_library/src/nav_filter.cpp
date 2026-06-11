#include <iostream>
#include <eigen3/Eigen/Dense>
#include "nav_filter.hpp"
#include "imu_model.hpp"
#include "twist_model.hpp"
#include "pose_model.hpp"

// Class responsible for the navigation filter operations 

NavFilter::NavFilter(const IMUModel& imu_model, const TwistModel& twist_model, const PoseModel& pose_model) :imu(imu_model), twist(twist_model), pose(pose_model)
{

}

NavFilter::NavFilter()
{

}

NavFilter::~NavFilter()
{
    
}

// Basicamente propaga os erros passados para os erros futuros, usando as matrizes jacobianas.
// Por exemplo, se você tem um erro de velocidade, esse erro multiplicado pelo tempo se torna um erro de posição;

void NavFilter::propagate_error(Eigen::Matrix<float, 6,1> imu_measurement)
{
    Eigen::Matrix<float, 18, 18> A_k = this->imu.get_jacobian(error_state, imu_measurement);
    Eigen::Matrix<float, 18, 18> B_k = this->imu.get_noise_jacobian(error_state);
    Eigen::Matrix<float, 18, 18> P_old = this->error_state.error_covariance_;

    this->error_state = A_k * this->error_state.get_error_vector();
    this->error_state.error_covariance_ = A_k * P_old * A_k.transpose() 
                                        + B_k * this->imu.covariance_ * B_k.transpose();
}

void NavFilter::updateEKF(const Eigen::MatrixXf& measurement, 
                          const Eigen::MatrixXf& measurement_covariance, 
                          const Eigen::MatrixXf& jacobian_matrix, 
                          const Eigen::MatrixXf& predicted_measurement, 
                          const Eigen::MatrixXf& noise_jacobian_matrix)
{

    // P_ --> Covariancia do estado de erro, mede o quão incerto o filtro está antes de receber a próxima medição;
    Eigen::Matrix<float, 18, 18> P_ = this->error_state.error_covariance_;

    // H_k1 --> Jacobiana da medição; 
    Eigen::MatrixXf H_k1 = jacobian_matrix;

    // S_k1 e M_k1 --> Incertezas do sensor; 
    Eigen::MatrixXf S_k1 = measurement_covariance;
    Eigen::MatrixXf M_k1 = noise_jacobian_matrix;

    // Cálculo do ganho de Kalman K_k1, que determina o quanto o filtro deve confiar na nova medição em relação ao estado predito;
    // Se a incerteza do sistema (P) for muito alta e o ruído do sensor (S) for baixo, K será alto (o filtro confiará mais no sensor);
    // Se o sensor for ruidoso (S é grande), K será pequeno (o filtro ignorará parte da medição e confiará mais na sua própria inércia);
    

    // Eigen::MatrixXf K_k1 = P_ * H_k1.transpose() * (H_k1 * P_ * H_k1.transpose() + M_k1 * S_k1 * M_k1.transpose()).inverse();

    Eigen::MatrixXf S = H_k1 * P_ * H_k1.transpose() + M_k1 * S_k1 * M_k1.transpose();
    Eigen::MatrixXf K_k1 = P_ * H_k1.transpose() * S.inverse();


    // Erro entre o que foi lido e o que ele esperava ler;
    Eigen::MatrixXf error_z = measurement - predicted_measurement;

    // Atualização do estado de erro e sua covariancia com base na medição recebida;
    this->error_state = this->error_state.get_error_vector() + K_k1 * (error_z - H_k1 * this->error_state.get_error_vector());
    this->error_state.error_covariance_ = (Eigen::Matrix<float, 18, 18>::Identity() - K_k1 * H_k1) * P_;
}

// Declarando as matrizes necessárias para o EKF, a base dessa função é relacionar o estado medido com o estado predito,
// usando as matrizes jacobianas e de covariancia para calcular o ganho de Kalman e atualizar o estado de erro e sua covariancia. 

    // Jacobian -->   matriz que relaciona pequenas mudanças no estado de erro com mudanças na medição prevista.
    // Predicted Measurement -->   é o que o modelo de movimento espera medir, dado o estado atual do sistema.
    // Measurement Covariance -->   representa a incerteza associada às medições do sensor.
    // Noise Jacobian -->   matriz que relaciona o ruído do sensor com o estado de erro.


void NavFilter::update_pose(Eigen::Matrix<float, 6,1> pose_measurement)
{
    std::cerr << "1" << std::endl;
    Eigen::Matrix<float, 6, 18> jacobian = pose.get_jacobian(error_state);

    std::cerr << "2" << std::endl;
    Eigen::Matrix<float, 6, 1> predicted_measurement = pose.get_measurement(error_state);

    std::cerr << "3 - covariance size: " << pose.get_covariance().rows() << "x" << pose.get_covariance().cols() << std::endl;
    Eigen::Matrix<float, 6, 6> measurement_covariance = pose.get_covariance().cast<float>();

    std::cerr << "4" << std::endl;
    // era 6, 18
    Eigen::Matrix<float, 6, 18> noise_jacobian = pose.get_noise_jacobian(error_state);

    this->updateEKF(pose_measurement, measurement_covariance, jacobian, predicted_measurement, noise_jacobian);
    state.position_ = state.position_ + error_state.error_position_;
}

// Essa função atualiza o filtro com as medições de velocidade linear e angular, vindas do DVL. No Sensor Fusion é o DLVupdate que faz isso.

void NavFilter::update_twist(Eigen::Matrix<float, 6,1> twist_measurement)
{ 
    std::cerr << "TWIST 1" << std::endl;
    Eigen::Matrix<float, 6, 18> jacobian = twist.get_jacobian(error_state, state);
    std::cerr << "TWIST 2" << std::endl;
    Eigen::Matrix<float, 6, 1> predicted_measurement = twist.get_measurement(error_state);
    std::cerr << "TWIST 3" << std::endl;
    Eigen::Matrix<float, 6, 6> measurement_covariance = twist.get_covariance().cast<float>();
    std::cerr << "TWIST 4" << std::endl;
    // era 6, 18
    Eigen::Matrix<float, 6, 6> noise_jacobian = twist.get_noise_jacobian(error_state);
    std::cerr << "TWIST 5" << std::endl;
    this->updateEKF(twist_measurement, measurement_covariance, jacobian, predicted_measurement, noise_jacobian);
    state.velocity_ = state.velocity_ + error_state.error_velocity_;
}

// Update the filter with IMU measurements
// Função de mechanization serve para integrar as leituras do IMU e atualizar o estado nominal do seu ROV
// A função de propagate_error calcula a matriz de covariancia do estado de erro com base nas medições do IMU

void NavFilter::update_imu(Eigen::Matrix<float, 6,1> imu_measurement)
{
    std::cerr << "IMU 1" << std::endl;
    mechanization(imu_measurement);
    std::cerr << "IMU 2" << std::endl;
    propagate_error(imu_measurement);
    std::cerr << "IMU 3" << std::endl;
}

// Mechanization function to update the nominal state based on IMU measurements

void NavFilter::mechanization(Eigen::Matrix<float, 6,1> imu_measurement)
{

    // Pega os dados referentes ao acelerometro, rotaciona para o sistema de referencia NED e adiciona a gravidade: 
    Eigen::Matrix<float, 3, 1> velocity_dot = this->C_n_b * imu_measurement.head<3>() + this->gravity;

    // Tratamento do giroscopio com metodo de Runge-Kutta de ordem 4 (?)
    // alfa_delta é a variação angular simples;
    Eigen::Matrix<float, 3, 1> alfa_delta = (imu_measurement.tail<3>() + this->last_gyro) / 2 * this->imu.update_time_;

    // beta_delta é o cálculo de compensação de coning (rotação) --> Produto vetorial (cross) para corrigir erros;
    Eigen::Matrix<float, 3, 1> beta_delta = 0.5 * (this->alfa + 1.0 / 6.0 * this->last_alfa_delta).cross(alfa_delta);

    this->alfa += alfa_delta;
    this->beta += beta_delta;

    this->last_gyro = imu_measurement.tail<3>();
    this->last_alfa = this->alfa;
    this->last_beta = this->beta;
    this->last_alfa_delta = alfa_delta;

    // Atualiza o estado nominal com as novas leituras do IMU, usando física básica de movimento
    // ou seja, integrando aceleração para obter velocidade e integrando velocidade para obter posição

    this->state.position_ += this->state.velocity_ * this->imu.update_time_;
    this->state.velocity_ += velocity_dot * this->imu.update_time_;
    this->state.orientation_ = this->alfa + this->beta + this->last_orientation;

    this->C_n_b = Eigen::AngleAxisf(this->state.orientation_.norm(), this->state.orientation_.normalized()).toRotationMatrix();
}
State NavFilter::get_state()
{
    return state;
}