#ifndef STATE_HPP
#define STATE_HPP

#include <eigen3/Eigen/Dense>
#include <vector>

class State
{
private:

public:

    Eigen::Vector<float, 3> position_;
    Eigen::Vector<float, 3> velocity_;
    Eigen::Vector<float, 3> orientation_;
    Eigen::Vector<float, 3> angular_velocity_;

    Eigen::Matrix<float, 12, 12> covariance_;
    
    State(Eigen::Vector<float, 12>state_vector, Eigen::Matrix<float, 12, 12> covariance);

    Eigen::Matrix<float, 12, 12> get_covariance();
    Eigen::Vector<float, 12> get_state_vector();

    State();
    ~State();
};

#endif