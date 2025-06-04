#ifndef TWIST_MODEL_HPP
#define TWIST_MODEL_HPP

#include <eigen3/Eigen/Dense>

class TwistModel
{
private:
    float update_time;
    Eigen::Matrix4f transform_twist_imu;
    Eigen::Vector3f twist_noise;

public:
    TwistModel(Eigen::Matrix4f transform_imu, Eigen::Vector3f noise, float dt);
    TwistModel(const TwistModel& other);
    ~TwistModel();
};

#endif // TWIST_MODEL_HPP
