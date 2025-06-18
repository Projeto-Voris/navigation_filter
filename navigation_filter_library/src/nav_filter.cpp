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

void NavFilter::update_pose(Eigen::Matrix<float, 6,1> pose_measurement)
{
    Eigen::Matrix<float, 6, 18> jacobian = pose.get_jacobian(error_state);
    Eigen::Matrix<float, 6, 1> predicted_measurement = pose.get_measurement(error_state);
    Eigen::Matrix<float, 6, 6> measurement_covariance = pose.get_covariance().cast<float>();
    Eigen::Matrix<float, 6, 18> noise_jacobian = pose.get_noise_jacobian(error_state);

    this->updateEKF(pose_measurement, measurement_covariance, jacobian, predicted_measurement, noise_jacobian);
    state.position_ = state.position_ + error_state.error_position_;
}

void NavFilter::update_twist(Eigen::Matrix<float, 6,1> twist_measurement)
{
    Eigen::Matrix<float, 6, 18> jacobian = twist.get_jacobian(error_state);
    Eigen::Matrix<float, 6, 1> predicted_measurement = twist.get_measurement(error_state);
    Eigen::Matrix<float, 6, 6> measurement_covariance = twist.get_covariance().cast<float>();
    Eigen::Matrix<float, 6, 18> noise_jacobian = twist.get_noise_jacobian(error_state);

    this->updateEKF(twist_measurement, measurement_covariance, jacobian, predicted_measurement, noise_jacobian);

    state.velocity_ = state.velocity_ + error_state.error_velocity_;
}
void NavFilter::update_imu(Eigen::Matrix<float, 6,1> imu_measurement)
{
    mechanization(imu_measurement);
    propagate_error(imu_measurement);
}

void NavFilter::mechanization(Eigen::Matrix<float, 6,1> imu_measurement)
{
    Eigen::Matrix<float, 3, 1> velocity_dot = this->C_n_b * imu_measurement.head<3>() + this->gravity;

    Eigen::Matrix<float, 3, 1> alfa_delta = (imu_measurement.tail<3>() + this->last_gyro) / 2 * this->imu.update_time_;
    Eigen::Matrix<float, 3, 1> beta_delta = 0.5 * (this->alfa + 1.0 / 6.0 * this->last_alfa_delta).cross(alfa_delta);

    this->alfa += alfa_delta;
    this->beta += beta_delta;

    this->last_gyro = imu_measurement.tail<3>();
    this->last_alfa = this->alfa;
    this->last_beta = this->beta;
    this->last_alfa_delta = alfa_delta;

    this->state.position_ += this->state.velocity_ * this->imu.update_time_;
    this->state.velocity_ += velocity_dot * this->imu.update_time_;
    this->state.orientation_ = this->alfa + this->beta + this->last_orientation;

    this->C_n_b = Eigen::AngleAxisf(this->state.orientation_.norm(), this->state.orientation_.normalized()).toRotationMatrix();
}