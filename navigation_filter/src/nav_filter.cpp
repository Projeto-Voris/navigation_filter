#include <eigen3/Eigen/Dense>

class NavFilter
{
private:
    // IMU parameters
    float imu_update_time;
    Eigen::Matrix4f transform_imu_base;
    Eigen::Vector3f accelerometer_random_walk;
    Eigen::Vector3f gyroscope_random_walk;
    
    Eigen::Vector3f accelerometer_noise;
    Eigen::Vector3f gyroscope_noise;

    //Twist parameters
    float twist_update_time;
    Eigen::Matrix4f transform_twist_imu;
    Eigen::Vector3f twist_noise;

    //Pose parameters
    float pose_update_time;
    Eigen::Matrix4f transform_pose_imu;
    Eigen::Vector3f pose_noise;

    //Filter parameters
    Eigen::Matrix<float, 18, 1> error;
    Eigen::Matrix<float, 18, 18> error_covariance;
    Eigen::Matrix<float, 3, 1> corr_T;
    Eigen::Matrix<float, 3, 1> corr_noise;
public:
    NavFilter(/* args */);
    ~NavFilter();
};

NavFilter::NavFilter(/* args */)
{
}

NavFilter::~NavFilter()
{
}
