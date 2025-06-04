#include <iostream>
#include <eigen3/Eigen/Dense>
#include "nav_filter.hpp"
#include "imu_model.cpp"
#include "twist_model.cpp"
#include "pose_model.cpp"


NavFilter::NavFilter(const IMUModel& imu_model, const TwistModel& twist_model, const PoseModel& pose_model) :imu(imu_model), twist(twist_model), pose(pose_model)
{
    
}

NavFilter::~NavFilter()
{
}
