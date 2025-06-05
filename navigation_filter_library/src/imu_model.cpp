#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>
#include "imu_model.hpp"

IMUModel::IMUModel(const IMUModel& other)
    : update_time(other.update_time),
      transform_imu_base(other.transform_imu_base),
      accelerometer_random_walk(other.accelerometer_random_walk),
      gyroscope_random_walk(other.gyroscope_random_walk),
      accelerometer_noise(other.accelerometer_noise),
      gyroscope_noise(other.gyroscope_noise)
{
}
IMUModel& IMUModel::operator=(const IMUModel& other)
{
    update_time = other.update_time;
    return *this;
}
IMUModel::IMUModel(const Eigen::Matrix4f& transform_base,
    std::vector<double> acc_random_walk,
    std::vector<double> gyro_random_walk,
    std::vector<double> acc_noise,
    std::vector<double> gyro_noise,
    float dt): transform_imu_base(transform_base),update_time(dt)
{
    accelerometer_random_walk = Eigen::Map<Eigen::Vector3d>(acc_random_walk.data(), acc_random_walk.size());
    gyroscope_random_walk = Eigen::Map<Eigen::Vector3d>(gyro_random_walk.data(), gyro_random_walk.size());
    accelerometer_noise = Eigen::Map<Eigen::Vector3d>(acc_noise.data(), acc_noise.size());
    gyroscope_noise = Eigen::Map<Eigen::Vector3d>(gyro_noise.data(), gyro_noise.size());
}
IMUModel::IMUModel()
{}
IMUModel::~IMUModel()
{
}
