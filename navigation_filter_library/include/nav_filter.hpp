#ifndef NAV_FILTER_HPP
#define NAV_FILTER_HPP

#include <eigen3/Eigen/Dense>
#include "imu_model.hpp"
#include "twist_model.hpp"
#include "pose_model.hpp"
#include "error_state.hpp"
#include "state.hpp"

class NavFilter
{
private:
    IMUModel imu;
    TwistModel twist;
    PoseModel pose;

    const Eigen::Vector3f gravity = Eigen::Vector3f(0, 0, 9.81);

    ErrorState error_state;
    State state;

    Eigen::Matrix<float, 3, 3> C_n_b;

    Eigen::Vector3f last_gyro;
    Eigen::Vector3f last_alfa;
    Eigen::Vector3f last_beta;
    Eigen::Vector3f last_alfa_delta;
    Eigen::Vector3f alfa;
    Eigen::Vector3f beta;
    Eigen::Vector3f last_orientation;


public:
    NavFilter(const IMUModel& imu_model, const TwistModel& twist_model, const PoseModel& pose_model);
    NavFilter();
    ~NavFilter();
    
    void propagate_error(Eigen::Matrix<float, 6,1> imu_measurement);
    void updateEKF(const Eigen::MatrixXf& measurement, 
        const Eigen::MatrixXf& measurement_covariance, 
        const Eigen::MatrixXf& jacobian_matrix, 
        const Eigen::MatrixXf& predicted_measurement, 
        const Eigen::MatrixXf& noise_jacobian_matrix);
    void update_pose(Eigen::Matrix<float, 6,1> pose_measurement);
    void update_twist(Eigen::Matrix<float, 6,1> twist_measurement);
    void update_imu(Eigen::Matrix<float, 6,1> imu_measurement);
    void mechanization(Eigen::Matrix<float, 6,1> imu_measurement);
};

#endif // NAV_FILTER_HPP
