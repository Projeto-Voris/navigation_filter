#include <eigen3/Eigen/Dense>
#include "pose_model.hpp"

PoseModel::PoseModel(Eigen::Matrix4f transform_imu, Eigen::Vector3f noise, float dt)
{
    transform_pose_imu = transform_imu;
    pose_noise = noise;
    update_time = dt;
}
PoseModel::PoseModel(const PoseModel& other)
    : update_time(other.update_time),
      transform_pose_imu(other.transform_pose_imu),
      pose_noise(other.pose_noise)
{
}
PoseModel::~PoseModel()
{
}
