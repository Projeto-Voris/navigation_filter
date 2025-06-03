#include <eigen3/Eigen/Dense>

class PoseModel
{
private:
    //Twist parameters
    float update_time;
    Eigen::Matrix4f transform_twist_imu;
    Eigen::Vector3f pose_noise;

public:
    PoseModel(Eigen::Matrix4f transform_imu, Eigen::Vector3f noise, float dt);
    ~PoseModel();
};

PoseModel::PoseModel(Eigen::Matrix4f transform_imu, Eigen::Vector3f noise, float dt)
{
    transform_twist_imu = transform_imu;
    pose_noise = noise;
    update_time = dt;
}

PoseModel::~PoseModel()
{
}
