#include "pose_model.hpp"

Eigen::MatrixXf PoseModel::get_jacobian(ErrorState state)
{
    Eigen::Matrix<float, 6, 18> jacobian = Eigen::Matrix<float, 6, 18>::Zero();

    jacobian(0, 0)  = -1;
    jacobian(1, 1)  = -1;
    jacobian(2, 2)  = -1;
    jacobian(3, 6)  = -1;
    jacobian(4, 7)  = -1;
    jacobian(5, 8)  = -1;

    return jacobian;
}
Eigen::MatrixXf PoseModel::get_noise_jacobian(ErrorState state)
{
    Eigen::Matrix<float, 6, 18> jacobian = Eigen::Matrix<float, 6, 18>::Zero();

    jacobian(0, 0)  = -1;
    jacobian(1, 1)  = -1;
    jacobian(2, 2)  = -1;
    jacobian(3, 6)  = -1;
    jacobian(4, 7)  = -1;
    jacobian(5, 8)  = -1;

    /* jacobian = [[-1,  0,  0, 0, 0, 0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                   [ 0, -1,  0, 0, 0, 0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                   [ 0,  0, -1, 0, 0, 0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                   [ 0,  0,  0, 0, 0, 0, -1,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                   [ 0,  0,  0, 0, 0, 0,  0, -1,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                   [ 0,  0,  0, 0, 0, 0,  0,  0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0]]; */

    return jacobian;
}
Eigen::MatrixXf PoseModel::get_measurement(ErrorState state)
{
    Eigen::Vector<float, 18> error_state = state.get_error_vector();
    Eigen::Vector<float, 3> error_position = error_state.segment(0,3);
    Eigen::Vector<float, 3> error_orientation = error_state.segment(7,3);

    Eigen::VectorXf result(error_position.size() + error_orientation.size());

    result << error_position, error_orientation;
    
    return -result;
}