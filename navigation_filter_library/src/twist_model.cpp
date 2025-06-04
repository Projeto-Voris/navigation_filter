#include <eigen3/Eigen/Dense>
#include "twist_model.hpp"  

TwistModel::TwistModel(Eigen::Matrix4f transform_imu, Eigen::Vector3f noise, float dt)
{
    transform_twist_imu = transform_imu;
    twist_noise = noise;
    update_time = dt;
}
TwistModel::TwistModel(const TwistModel& other)
    : update_time(other.update_time),
      transform_twist_imu(other.transform_twist_imu),
      twist_noise(other.twist_noise)
{
}
TwistModel::~TwistModel()
{
}
