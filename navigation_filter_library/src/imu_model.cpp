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

IMUModel::~IMUModel()
{
}
