#include <iostream>
#include <eigen3/Eigen/Dense>
#include "imu_model.cpp"
#include "twist_model.cpp"
#include "pose_model.cpp"

class NavFilter
{
private:
    IMUModel imu;
    //TwistModel twist;
    //PoseModel pose;

    const Eigen::Vector3f gravity = Eigen::Vector3f(0, 0, 9.81);

    //Filter parameters
    Eigen::Matrix<float, 18, 1> error;
    Eigen::Matrix<float, 18, 18> error_covariance;
    Eigen::Matrix<float, 3, 1> corr_T;
    Eigen::Matrix<float, 3, 1> corr_noise;

public:
    NavFilter(const IMUModel& imu_model);
    ~NavFilter();
};

NavFilter::NavFilter(const IMUModel& imu_model) :imu(imu_model)
{
    //twist = twist_model;
    //pose = pose_model;
}

NavFilter::~NavFilter()
{
}
