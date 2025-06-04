#include <eigen3/Eigen/Dense>

class IMUModel
{
private:
    // IMU parameters
    float update_time;
    Eigen::Matrix4f transform_imu_base;
    Eigen::Vector3f accelerometer_random_walk;
    Eigen::Vector3f gyroscope_random_walk;
    
    Eigen::Vector3f accelerometer_noise;
    Eigen::Vector3f gyroscope_noise;
public:
    IMUModel(Eigen::Matrix4f transform_base, Eigen::Vector3f acc_random_walk, Eigen::Vector3f gyro_random_walk, Eigen::Vector3f acc_noise, Eigen::Vector3f gyro_noise, float dt);
    IMUModel(const IMUModel& other);
    ~IMUModel();
};

IMUModel::IMUModel(const Eigen::Matrix4f& transform_base,
    const Eigen::Vector3f& acc_random_walk,
    const Eigen::Vector3f& gyro_random_walk,
    const Eigen::Vector3f& acc_noise,
    const Eigen::Vector3f& gyro_noise,
    float dt)
: transform_imu_base(transform_base),
accelerometer_random_walk(acc_random_walk),
gyroscope_random_walk(gyro_random_walk),
accelerometer_noise(acc_noise),
gyroscope_noise(gyro_noise),
update_time(dt)
{
}

IMUModel::IMUModel(const IMUModel& other)
    : update_time(other.update_time),
      transform_imu_base(other.transform_imu_base),
      accelerometer_random_walk(other.accelerometer_random_walk),
      gyroscope_random_walk(other.gyroscope_random_walk),
      accelerometer_noise(other.accelerometer_noise),
      gyroscope_noise(other.gyroscope_noise)
{
}

IMUModel::~IMUModel()
{
}
