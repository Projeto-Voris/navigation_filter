#ifndef POSE_MODEL_HPP
#define POSE_MODEL_HPP

#include <eigen3/Eigen/Dense>

class PoseModel
{
private:
    //Twist parameters
    float update_time;
    Eigen::Matrix4f transform_pose_imu;
    Eigen::Vector3f pose_noise;

public:
    PoseModel(Eigen::Matrix4f transform_imu, Eigen::Vector3f noise, float dt);
    PoseModel(const PoseModel& other);
    ~PoseModel();
};

#endif