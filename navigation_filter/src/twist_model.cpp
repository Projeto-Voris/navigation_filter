#include <eigen3/Eigen/Dense>

class TwistModel
{
private:
    //Twist parameters
    float update_time;
    Eigen::Matrix4f transform_twist_imu;
    Eigen::Vector3f twist_noise;

public:
    TwistModel(Eigen::Matrix4f transform_imu, Eigen::Vector3f noise, float dt);
    ~TwistModel();
};

TwistModel::TwistModel(Eigen::Matrix4f transform_imu, Eigen::Vector3f noise, float dt)
{
    transform_twist_imu = transform_imu;
    twist_noise = noise;
    update_time = dt;
}

TwistModel::~TwistModel()
{
}
