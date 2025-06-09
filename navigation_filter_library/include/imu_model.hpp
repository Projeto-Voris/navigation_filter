#ifndef IMU_MODEL_HPP
#define IMU_MODEL_HPP

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>

#include "proccess_model.hpp"

class IMUModel : public ProccessModel
{
private:

public:
    using ProccessModel::ProccessModel;
    IMUModel(const Eigen::Matrix4f& transform_base,
             std::vector<double> acc_random_walk,
             std::vector<double> gyro_random_walk,
             std::vector<double> acc_noise,
             std::vector<double> gyro_noise,
             std::vector<double> corr_noise,
             float dt);
};

#endif // IMU_MODEL_HPP
