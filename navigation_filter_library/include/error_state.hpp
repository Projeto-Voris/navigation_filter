#ifndef ERROR_STATE_HPP
#define ERROR_STATE_HPP

#include <eigen3/Eigen/Dense>
#include <vector>

class ErrorState
{
private:
    
    Eigen::Vector<float, 3> error_position_;
    Eigen::Vector<float, 3> error_velocity_;
    Eigen::Vector<float, 3> error_orientation_;

    Eigen::Vector<float, 3> error_ba_;
    Eigen::Vector<float, 3> error_bw_;
    Eigen::Vector<float, 3> bv_;

    Eigen::Matrix<float, 18, 18> error_covariance_;

public:
    ErrorState(Eigen::Matrix<float, 18, 18> error_covariance);

    Eigen::Matrix<float, 18, 18> get_covariance();
    Eigen::Vector<float, 18> get_error_vector();

    ErrorState();
    ~ErrorState();
};

#endif