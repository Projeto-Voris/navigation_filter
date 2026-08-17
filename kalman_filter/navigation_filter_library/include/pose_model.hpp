#ifndef POSE_MODEL_HPP
#define POSE_MODEL_HPP

#include <eigen3/Eigen/Dense>
#include <vector>
#include "sensor_model.hpp"

class PoseModel : public SensorModel
{
    private:


    public:
        using SensorModel::SensorModel;
        Eigen::MatrixXf get_jacobian(ErrorState state) override;
        Eigen::MatrixXf get_noise_jacobian(ErrorState state) override;
        Eigen::MatrixXf get_measurement(ErrorState state) override;
};

#endif