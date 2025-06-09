#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>
#include "imu_model.hpp"

IMUModel::IMUModel(const Eigen::Matrix4f& transform_base,
    std::vector<double> acc_random_walk,
    std::vector<double> gyro_random_walk,
    std::vector<double> acc_noise,
    std::vector<double> gyro_noise,
    std::vector<double> corr_noise,
    float dt)
{
    transform_ = transform_base;
    update_time_ = dt;
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
