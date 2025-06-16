#include <iostream>
#include <eigen3/Eigen/Dense>
#include "nav_filter.hpp"
#include "imu_model.cpp"
#include "twist_model.cpp"
#include "pose_model.cpp"


NavFilter::NavFilter(const IMUModel& imu_model, const TwistModel& twist_model, const PoseModel& pose_model) :imu(imu_model), twist(twist_model), pose(pose_model)
{
}
NavFilter::NavFilter()
{}
NavFilter::~NavFilter()
{
}

void NavFilter::propagate_error(Eigen::Matrix<float, 6,1> imu_measurement)
{
    Eigen::Matrix<float, 18, 18> A_k = this->imu.get_jacobian(error_state, imu_measurement);
    this->error_state = A_k * this->error_state.get_error_vector();
    Eigen::Matrix<float, 18, 18> B_k = this->imu.get_noise_jacobian(error_state);
    this->error_state.error_covariance_ = A_k*this->error_state.error_covariance_*A_k.transpose() + B_k*this->imu.covariance_*B_k.transpose();
}
void NavFilter::updateEKF(const Eigen::MatrixXf& measurement, 
                          const Eigen::MatrixXf& measurement_covariance, 
                          const Eigen::MatrixXf& jacobian_matrix, 
                          const Eigen::MatrixXf& predicted_measurement, 
                          const Eigen::MatrixXf& noise_jacobian_matrix)
{
    Eigen::Matrix<float, 18, 18> P_ = this->error_state.error_covariance_;
    Eigen::MatrixXf H_k1 = jacobian_matrix;
    Eigen::MatrixXf S_k1 = measurement_covariance;
    Eigen::MatrixXf M_k1 = noise_jacobian_matrix;

    Eigen::MatrixXf K_k1 = P_ * H_k1.transpose() * 
                                       (H_k1 * P_ * H_k1.transpose() + M_k1 * S_k1 * M_k1.transpose()).inverse();
    Eigen::MatrixXf error_z = measurement - predicted_measurement;

    this->error_state = this->error_state.get_error_vector() + K_k1 * (error_z - H_k1 * this->error_state.get_error_vector());
    this->error_state.error_covariance_ = (Eigen::Matrix<float, 18, 18>::Identity() - K_k1 * H_k1) * P_;
}