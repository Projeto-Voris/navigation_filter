#ifndef POSE_MODEL_HPP
#define POSE_MODEL_HPP

#include <eigen3/Eigen/Dense>
#include <vector>

class PoseModel
{
private:
    //Twist parameters
    float update_time;
    Eigen::Matrix4f transform_pose_imu;
    Eigen::Vector3d pose_noise;

public:
    PoseModel(Eigen::Matrix4f transform_imu, Eigen::Vector3d noise, float dt);
    PoseModel(Eigen::Matrix4f transform_imu, std::vector<double> noise, float dt);
    PoseModel(const PoseModel& other);
    PoseModel();
    ~PoseModel();
};

#endif