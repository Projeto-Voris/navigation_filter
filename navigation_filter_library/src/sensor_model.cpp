#include <eigen3/Eigen/Dense>

#include "sensor_model.hpp"

SensorModel::SensorModel(Eigen::Matrix4f transform, Eigen::VectorXd noise, float dt)
{
    transform = transform;
    covariance = noise.asDiagonal();
    update_time = dt;
}
SensorModel::SensorModel(const SensorModel& other)
    : update_time(other.update_time),
      transform(other.transform),
      covariance(other.covariance)
{}
SensorModel::SensorModel(Eigen::Matrix4f transform, std::vector<double> noise, float dt):transform(transform), update_time(dt)
{
    covariance = Eigen::Map<Eigen::VectorXd>(noise.data(), noise.size()).asDiagonal();
}
SensorModel::SensorModel()
{}
SensorModel::~SensorModel()
{}

float SensorModel::get_update_time()
{
    return update_time;
}
Eigen::Matrix4f SensorModel::get_transform()
{
    return transform;
}
Eigen::MatrixXf SensorModel::get_covariance()
{
    return covariance;
}