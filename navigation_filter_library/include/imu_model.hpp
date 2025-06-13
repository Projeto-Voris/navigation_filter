#ifndef IMU_MODEL_HPP
#define IMU_MODEL_HPP

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>

#include "proccess_model.hpp"

class IMUModel : public ProccessModel
{
private:
    Eigen::Matrix3d corr_transform_;
public:
    using ProccessModel::ProccessModel;
    IMUModel(const Eigen::Matrix4f& transform_base,
             std::vector<double> acc_random_walk,
             std::vector<double> gyro_random_walk,
             std::vector<double> acc_noise,
             std::vector<double> gyro_noise,
             std::vector<double> corr_noise,
             Eigen::Matrix3d corr_transform,
             float dt);
    Eigen::MatrixXf get_jacobian(ErrorState state, Eigen::VectorXf control_vector) override;
    Eigen::MatrixXf get_noise_jacobian(ErrorState state) override;
};

#endif // IMU_MODEL_HPP
