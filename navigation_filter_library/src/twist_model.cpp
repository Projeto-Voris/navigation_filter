#include "twist_model.hpp"

Eigen::MatrixXf TwistModel::get_jacobian(ErrorState e_state, State state)
{
    float lambda1 = e_state.error_orientation_(0);
    float lambda2 = e_state.error_orientation_(1);
    float lambda3 = e_state.error_orientation_(2);
    float bv1 = e_state.bv_(0);
    float bv2 = e_state.bv_(1);
    float bv3 = e_state.bv_(2);
    float v_nom1 = state.velocity_(0);
    float v_nom2 = state.velocity_(1);
    float v_nom3 = state.velocity_(2);

    Eigen::Matrix<float, 6, 18> jacobian = Eigen::Matrix<float, 6, 18>::Zero();

    // Velocidade linear (linhas 0-2) — igual ao que tinha antes
    jacobian(0, 3)  = -1;
    jacobian(0, 6)  = bv2;
    jacobian(0, 7)  = bv3 - v_nom1;
    jacobian(0, 8)  = -v_nom2;
    jacobian(0, 15) = 1;
    jacobian(0, 16) = lambda1;
    jacobian(0, 17) = lambda2;

    jacobian(1, 4)  = -1;
    jacobian(1, 6)  = v_nom1 - bv1;
    jacobian(1, 8)  = bv3 - v_nom3;
    jacobian(1, 15) = -lambda1;
    jacobian(1, 16) = 1;
    jacobian(1, 17) = lambda3;

    jacobian(2, 5)  = -1;
    jacobian(2, 6)  = v_nom2;
    jacobian(2, 7)  = v_nom3 - bv1;
    jacobian(2, 8)  = -bv2;
    jacobian(2, 15) = -lambda2;
    jacobian(2, 16) = -lambda3;
    jacobian(2, 17) = 1;

    // Velocidade angular (linhas 3-5) — zeradas, sem observabilidade
    // jacobian(3..5, :) = 0  já por causa do ::Zero()

    return jacobian;
}

Eigen::MatrixXf TwistModel::get_noise_jacobian(ErrorState e_state)
{
    float lambda1 = e_state.error_orientation_(0);
    float lambda2 = e_state.error_orientation_(1);
    float lambda3 = e_state.error_orientation_(2);

    Eigen::Matrix<float, 6, 6> jacobian = Eigen::Matrix<float, 6, 6>::Zero();

    // Bloco linear (0-2): igual ao que tinha antes
    jacobian.block<3,3>(0,0) << 1,        lambda1,  lambda2,
                                -lambda1,  1,        lambda3,
                                -lambda2, -lambda3,  1;

    // Bloco angular (3-5): identidade — sem correção, passa ruído neutro
    jacobian.block<3,3>(3,3) = Eigen::Matrix3f::Identity();

    return jacobian;
}

Eigen::MatrixXf TwistModel::get_measurement(ErrorState state)
{
    Eigen::Vector<float, 18> error_state = state.get_error_vector();
    Eigen::Vector<float, 3> error_velocity    = error_state.segment<3>(3);
    Eigen::Vector<float, 3> error_orientation = error_state.segment<3>(6);

    Eigen::Matrix<float, 6, 1> result;
    result << error_velocity, error_orientation;

    return -result;
}