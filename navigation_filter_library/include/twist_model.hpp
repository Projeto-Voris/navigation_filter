#ifndef TWIST_MODEL_HPP
#define TWIST_MODEL_HPP

#include <vector>
#include <eigen3/Eigen/Dense>

class TwistModel
{
private:
    float update_time;
    Eigen::Matrix4f transform_twist_imu;
    Eigen::Vector3d twist_noise;

public:
    TwistModel(Eigen::Matrix4f transform_imu, Eigen::Vector3d noise, float dt);
    TwistModel(const TwistModel& other);
    TwistModel(Eigen::Matrix4f transform_imu, std::vector<double> noise, float dt);
    TwistModel();
    ~TwistModel();
};

#endif // TWIST_MODEL_HPP
