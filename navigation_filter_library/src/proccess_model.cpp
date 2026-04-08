#include <eigen3/Eigen/Dense>

#include "proccess_model.hpp"

ProccessModel::ProccessModel(const ProccessModel& other)
    : update_time_(other.update_time_),
      transform_(other.transform_),
      covariance_(other.covariance_)
{}
ProccessModel::ProccessModel(Eigen::Matrix4f transform, std::vector<double> noise, float dt):update_time_(dt), transform_(transform)
{
    std::vector<float> noise_float(noise.begin(), noise.end());
    covariance_ = Eigen::Map<Eigen::VectorXf>(noise_float.data(), noise.size()).asDiagonal();
}
ProccessModel::ProccessModel()
{}
ProccessModel::~ProccessModel()
{}

float ProccessModel::get_update_time()
{
    return update_time_;
}
Eigen::Matrix4f ProccessModel::get_transform()
{
    return transform_;
}
Eigen::MatrixXf ProccessModel::get_covariance()
{
    return covariance_;
}

// Define virtual functions with default implementations
Eigen::MatrixXf ProccessModel::get_jacobian(ErrorState /*state*/, Eigen::VectorXf /*(control_vector*/) {
    // Default implementation
    return Eigen::MatrixXf();
}
Eigen::MatrixXf ProccessModel::get_noise_jacobian(ErrorState /*state*/) {
    // Default implementation
    return Eigen::MatrixXf();
}

