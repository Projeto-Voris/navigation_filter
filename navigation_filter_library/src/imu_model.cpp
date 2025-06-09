#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>
#include "imu_model.hpp"

IMUModel::IMUModel(const Eigen::Matrix4f& transform_base,
    std::vector<double> acc_random_walk,
    std::vector<double> gyro_random_walk,
    std::vector<double> acc_noise,
    std::vector<double> gyro_noise,
    std::vector<double> corr_noise,
    Eigen::Matrix3f corr_transform,
    float dt)
{
    transform_ = transform_base;
    update_time_ = dt;
    corr_transform_ = corr_transform;
    Eigen::Matrix3d eye_block = Eigen::Matrix3d::Identity() * 10e-4;
    Eigen::MatrixXd acc_noise_diag = Eigen::Map<Eigen::VectorXd>(acc_noise.data(), acc_noise.size()).asDiagonal();
    Eigen::MatrixXd gyro_noise_diag = Eigen::Map<Eigen::VectorXd>(gyro_noise.data(), gyro_noise.size()).asDiagonal();
    Eigen::MatrixXd acc_rw_diag = Eigen::Map<Eigen::VectorXd>(acc_random_walk.data(), acc_random_walk.size()).asDiagonal();
    Eigen::MatrixXd gyro_rw_diag = Eigen::Map<Eigen::VectorXd>(gyro_random_walk.data(), gyro_random_walk.size()).asDiagonal();
    Eigen::MatrixXd corr_noise_diag = Eigen::Map<Eigen::VectorXd>(corr_noise.data(), corr_noise.size()).asDiagonal();

    // Calculate the total size of the covariance matrix
    int total_rows = eye_block.rows() + acc_noise_diag.rows() + gyro_noise_diag.rows() +
                     acc_rw_diag.rows() + gyro_rw_diag.rows() + corr_noise_diag.rows();
    int total_cols = eye_block.cols() + acc_noise_diag.cols() + gyro_noise_diag.cols() +
                     acc_rw_diag.cols() + gyro_rw_diag.cols() + corr_noise_diag.cols();

    // Initialize the covariance matrix
    covariance_ = Eigen::MatrixXd::Zero(total_rows, total_cols);

    // Fill the block diagonal matrix
    int current_row = 0;
    covariance_.block(current_row, current_row, eye_block.rows(), eye_block.cols()) = eye_block;
    current_row += eye_block.rows();

    covariance_.block(current_row, current_row, acc_noise_diag.rows(), acc_noise_diag.cols()) = acc_noise_diag;
    current_row += acc_noise_diag.rows();

    covariance_.block(current_row, current_row, gyro_noise_diag.rows(), gyro_noise_diag.cols()) = gyro_noise_diag;
    current_row += gyro_noise_diag.rows();

    covariance_.block(current_row, current_row, acc_rw_diag.rows(), acc_rw_diag.cols()) = acc_rw_diag;
    current_row += acc_rw_diag.rows();

    covariance_.block(current_row, current_row, gyro_rw_diag.rows(), gyro_rw_diag.cols()) = gyro_rw_diag;
    current_row += gyro_rw_diag.rows();

    covariance_.block(current_row, current_row, corr_noise_diag.rows(), corr_noise_diag.cols()) = corr_noise_diag;

}

Eigen::MatrixXf IMUModel::get_jacobian(ErrorState state, Eigen::VectorXf control_vector)
{
    Eigen::MatrixXf jacobian = Eigen::MatrixXf::Zero(18, 18);
    float dt = update_time_;
    
    float a1 = control_vector(0);
    float a2 = control_vector(1);
    float a3 = control_vector(2);

    float v1 = state.error_velocity_(0);
    float v2 = state.error_velocity_(1);
    float v3 = state.error_velocity_(2);

    float lambda1 = state.error_orientation_(0);
    float lambda2 = state.error_orientation_(1);
    float lambda3 = state.error_orientation_(2);

    float bw1 = state.error_bw_(0);
    float bw2 = state.error_bw_(1);
    float bw3 = state.error_bw_(2);

    float t1 = corr_transform_(0, 0);
    float t2 = corr_transform_(1, 1);
    float t3 = corr_transform_(2, 2);

    // Fill the jacobian matrix
    jacobian(0, 0) = 1;
    jacobian(0, 3) = dt;
    jacobian(1, 1) = 1;
    jacobian(1, 4) = dt;
    jacobian(2, 2) = 1;
    jacobian(2, 5) = dt;

    jacobian(3, 3) = 1 - dt;
    jacobian(3, 4) = -dt * lambda1;
    jacobian(3, 5) = -dt * lambda2;
    jacobian(3, 6) = -dt * (v2 - a1 * lambda3 + a2 * lambda2);
    jacobian(3, 7) = -dt * (a1 + v3 + a2 * lambda1 + 2 * a3 * lambda2);
    jacobian(3, 8) = -dt * (a2 - a1 * lambda1 + 2 * a3 * lambda3);

    jacobian(4, 3) = dt * lambda1;
    jacobian(4, 4) = 1 - dt;
    jacobian(4, 5) = -dt * lambda3;
    jacobian(4, 6) = dt * (a1 + v1 + 2 * a2 * lambda1 + a3 * lambda2);
    jacobian(4, 7) = dt * (a1 * lambda3 + a3 * lambda1);
    jacobian(4, 8) = -dt * (a3 + v3 - a1 * lambda2 - 2 * a2 * lambda3);

    jacobian(5, 3) = dt * lambda2;
    jacobian(5, 4) = dt * lambda3;
    jacobian(5, 5) = 1 - dt;
    jacobian(5, 6) = dt * (a2 - 2 * a1 * lambda1 + a3 * lambda3);
    jacobian(5, 7) = dt * (a3 + v1 - 2 * a1 * lambda2 - a2 * lambda3);
    jacobian(5, 8) = dt * (v2 - a2 * lambda2 + a3 * lambda1);

    jacobian(6, 6) = 1 - bw2 * dt;
    jacobian(6, 7) = -bw3 * dt;
    jacobian(6, 12) = -dt;
    jacobian(6, 13) = -dt * lambda1;
    jacobian(6, 14) = -dt * lambda2;

    jacobian(7, 6) = bw1 * dt;
    jacobian(7, 7) = 1;
    jacobian(7, 8) = -bw3 * dt;
    jacobian(7, 12) = dt * lambda1;
    jacobian(7, 13) = -dt;
    jacobian(7, 14) = -dt * lambda3;

    jacobian(8, 7) = bw1 * dt;
    jacobian(8, 8) = bw2 * dt + 1;
    jacobian(8, 12) = dt * lambda2;
    jacobian(8, 13) = dt * lambda3;
    jacobian(8, 14) = -dt;

    jacobian(15, 15) = 1 - dt / t1;
    jacobian(16, 16) = 1 - dt / t2;
    jacobian(17, 17) = 1 - dt / t3;

    return jacobian;
}
Eigen::MatrixXf get_noise_jacobian(ErrorState state)
{

}