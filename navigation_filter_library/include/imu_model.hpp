#ifndef IMU_MODEL_HPP
#define IMU_MODEL_HPP

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>

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
    IMUModel(const Eigen::Matrix4f& transform_base,
             const Eigen::Vector3f& acc_random_walk,
             const Eigen::Vector3f& gyro_random_walk,
             const Eigen::Vector3f& acc_noise,
             const Eigen::Vector3f& gyro_noise,
             float dt);

    IMUModel(const IMUModel& other);
    IMUModel& operator=(const IMUModel& other);
    ~IMUModel();
};

#endif // IMU_MODEL_HPP
