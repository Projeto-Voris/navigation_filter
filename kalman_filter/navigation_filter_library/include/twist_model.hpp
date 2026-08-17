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

        // Colored noise --> é um random walk relacionado com os erros que derivam/mudam com o tempo.
        // Exemplo: se o DVL tem um erro de 0.01 m/s, esse erro pode aumentar ou diminuir com o tempo, dependendo das condições do ambiente.
        // Esse tipo de ruído é modelado como um processo estocástico que evolui ao longo do tempo.
        // No caso do DVL, o ruído colorido pode representar variações na medição de velocidade devido a fatores como correntes marítimas, turbulência da água, ou mesmo interfer
        using SensorModel::SensorModel;
        Eigen::MatrixXf get_colored_jacobian(ErrorState e_state, State state);
        Eigen::MatrixXf get_noise_jacobian(ErrorState state) override;
        Eigen::MatrixXf get_measurement(ErrorState state) override;
};

#endif