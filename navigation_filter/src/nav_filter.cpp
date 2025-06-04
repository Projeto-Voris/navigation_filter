#include <eigen3/Eigen/Dense>
#include "imu_model.cpp"
#include "twist_model.cpp"
#include "pose_model.cpp"

class NavFilter
{
private:
    IMUModel imu;
    TwistModel twist;
    PoseModel pose;

    Eigen::Matrix<float, 3, 1> gravity;
    gravity << 0,0,9.81;

    //Filter parameters
    Eigen::Matrix<float, 18, 1> error;
    Eigen::Matrix<float, 18, 18> error_covariance;
    Eigen::Matrix<float, 3, 1> corr_T;
    Eigen::Matrix<float, 3, 1> corr_noise;

public:
    NavFilter(IMUModel imu_model, TwistModel twist_model, PoseModel pose_model);
    ~NavFilter();
};

NavFilter::NavFilter(IMUModel imu_model, TwistModel twist_model, PoseModel pose_model)
{
    imu = imu_model;
    twist = twist_model;
    pose = pose_model;
}

NavFilter::~NavFilter()
{
}
