#include "error_state.hpp"

ErrorState::ErrorState()
{
    // Initialize the error covariance matrix to identity
    error_covariance_.setIdentity();
    error_covariance_ *= 1e-8; // Scale it down to a small value
    error_position_ << 0,0,0;
    error_velocity_ << 0,0,0;
    error_orientation_ << 0,0,0;

    error_ba_ << 0,0,0;
    error_bw_ << 0,0,0;
    bv_ << 0,0,0;
}

ErrorState::ErrorState(Eigen::Matrix<float, 18, 18> error_covariance): error_covariance_(error_covariance)
{
    error_position_ << 0,0,0;
    error_velocity_ << 0,0,0;
    error_orientation_ << 0,0,0;

    error_ba_ << 0,0,0;
    error_bw_ << 0,0,0;
    bv_ << 0,0,0;
}
Eigen::Matrix<float, 18, 18> ErrorState::get_covariance()
{
    return error_covariance_;
}
Eigen::Vector<float, 18> ErrorState::get_error_vector()
{
    Eigen::Vector<float, 18> error_vector;
    error_vector << error_position_, error_velocity_, error_orientation_,
                    error_ba_, error_bw_, bv_;
    return error_vector;
}
ErrorState::~ErrorState()
{
    // Destructor
    // No dynamic memory to free, but can be used for cleanup if needed
}
