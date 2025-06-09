#include <eigen3/Eigen/Dense>

#include "sensor_model.hpp"

SensorModel::SensorModel(const SensorModel& other)
    : update_time_(other.update_time_),
      transform_(other.transform_),
      covariance_(other.covariance_)
{}
SensorModel::SensorModel(Eigen::Matrix4f transform, std::vector<double> noise, float dt):transform_(transform), update_time_(dt)
{
    covariance_ = Eigen::Map<Eigen::VectorXd>(noise.data(), noise.size()).asDiagonal();
}
SensorModel::SensorModel()
{}
SensorModel::~SensorModel()
{}

float SensorModel::get_update_time()
{
    return update_time_;
}
Eigen::Matrix4f SensorModel::get_transform()
{
    return transform_;
}
Eigen::MatrixXd SensorModel::get_covariance()
{
    return covariance_;
}

// Define virtual functions with default implementations
Eigen::MatrixXf SensorModel::get_jacobian(ErrorState state) {
    // Default implementation
    return Eigen::MatrixXf();
}
Eigen::MatrixXf SensorModel::get_noise_jacobian(ErrorState state) {
    // Default implementation
    return Eigen::MatrixXf();
}

Eigen::MatrixXf SensorModel::get_measurement(ErrorState state) {
    // Default implementation
    return Eigen::MatrixXf();
}
