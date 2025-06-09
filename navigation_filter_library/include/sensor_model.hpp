#ifndef SENSOR_MODEL_HPP
#define SENSOR_MODEL_HPP

#include <eigen3/Eigen/Dense>
#include <vector>
#include "error_state.hpp"

class SensorModel
{
private:
    //Twist parameters
    float update_time_;
    Eigen::Matrix4f transform_;
    Eigen::MatrixXd covariance_;

public:
    SensorModel(Eigen::Matrix4f transform, Eigen::VectorXd noise, float dt);
    SensorModel(Eigen::Matrix4f transform, std::vector<double> noise, float dt);
    SensorModel(const SensorModel& other);

    float get_update_time();
    Eigen::Matrix4f get_transform();
    Eigen::MatrixXd get_covariance();

    virtual Eigen::MatrixXf get_jacobian(ErrorState state);
    virtual Eigen::MatrixXf get_noise_jacobian(ErrorState state);
    virtual Eigen::MatrixXf get_measurement(ErrorState state);

    SensorModel();
    ~SensorModel();
};

#endif