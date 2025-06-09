#ifndef NAV_FILTER_HPP
#define NAV_FILTER_HPP

#include <eigen3/Eigen/Dense>
#include "imu_model.hpp"
#include "twist_model.hpp"
#include "pose_model.hpp"

class NavFilter
{
private:
    IMUModel imu;
    TwistModel twist;
    PoseModel pose;

    const Eigen::Vector3f gravity = Eigen::Vector3f(0, 0, 9.81);

    ErrorState error_state;

public:
    NavFilter(const IMUModel& imu_model, const TwistModel& twist_model, const PoseModel& pose_model);
    NavFilter();
    ~NavFilter();
    
    void propagate_error(Eigen::Matrix<float, 6,1> imu_measurement);

    Eigen::Matrix<float, 18, 18> get_proccess_jacobian(Eigen::Matrix<float, 6,1> imu_measurement);
    Eigen::Matrix<float, 18, 18> get_proccess_noise_jacobian();
};

#endif // NAV_FILTER_HPP
