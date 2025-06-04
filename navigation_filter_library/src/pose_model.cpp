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
