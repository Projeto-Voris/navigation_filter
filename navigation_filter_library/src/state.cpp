#include "state.hpp"

State::State(Eigen::Vector<float, 12>state_vector, Eigen::Matrix<float, 12, 12> covariance) : covariance_(covariance)
{
    position_ << 0, 0, 0;
    velocity_ << 0, 0, 0;
    orientation_ << 0, 0, 0;
    angular_velocity_ << 0,0,0;
}

Eigen::Matrix<float, 12, 12> State::get_covariance()
{
    return covariance_;
}
Eigen::Vector<float, 12> State::get_state_vector()
{
    Eigen::Vector<float, 12> vector;
    vector << position_, velocity_, orientation_, angular_velocity_;
    return vector;
}
