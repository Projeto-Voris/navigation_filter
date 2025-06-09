#ifndef PROCCESS_MODEL_HPP
#define PROCCESS_MODEL_HPP

#include <eigen3/Eigen/Dense>
#include <vector>
#include "error_state.hpp"

class ProccessModel
{
private:
    //Twist parameters
    float update_time_;
    Eigen::Matrix4f transform_;
    Eigen::MatrixXd covariance_;

public:
    ProccessModel(Eigen::Matrix4f transform, Eigen::VectorXd noise, float dt);
    ProccessModel(Eigen::Matrix4f transform, std::vector<double> noise, float dt);
    ProccessModel(const ProccessModel& other);

    float get_update_time();
    Eigen::Matrix4f get_transform();
    Eigen::MatrixXd get_covariance();

    virtual Eigen::MatrixXf get_jacobian(ErrorState state, Eigen::VectorXf control_vector);
    virtual Eigen::MatrixXf get_noise_jacobian(ErrorState state);

    ProccessModel();
    ~ProccessModel();
};

#endif