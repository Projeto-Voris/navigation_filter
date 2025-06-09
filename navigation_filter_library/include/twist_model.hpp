#ifndef TWIST_MODEL_HPP
#define TWIST_MODEL_HPP

#include <eigen3/Eigen/Dense>
#include <vector>
#include "sensor_model.hpp"
#include "state.hpp"

class TwistModel : public SensorModel
{
    private:


    public:
        using SensorModel::SensorModel;
        Eigen::MatrixXf get_colored_jacobian(ErrorState e_state, State state);
        Eigen::MatrixXf get_noise_jacobian(ErrorState state) override;
        Eigen::MatrixXf get_measurement(ErrorState state) override;
};

#endif