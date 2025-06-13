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
    /*this->error = A_k * this->error_state.get_error_vector();
    Eigen::Matrix<float, 18, 18> B_k = this->get_proccess_noise_jacobian();
    this->error_covariance = A_k*this->error_covariance*A_k.transpose() + B_k*this->proccess_covariance*B_k.transpose();*/
}

Eigen::Matrix<float, 18, 18> NavFilter::get_proccess_jacobian(Eigen::Matrix<float, 6,1> imu_measurement)
{

}

Eigen::Matrix<float, 18, 18> NavFilter::get_proccess_noise_jacobian()
{

}