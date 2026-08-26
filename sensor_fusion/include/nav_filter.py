import numpy as np
from include.error_state import ErrorState
from include.state import State


def angle_axis_to_rotation_matrix(angle: float, axis: np.ndarray) -> np.ndarray:
    """
    Equivalente a Eigen::AngleAxisf(angle, axis).toRotationMatrix().
    Fórmula de Rodrigues. 'axis' deve ser unitário.
    """
    if angle == 0.0 or np.linalg.norm(axis) == 0.0:
        return np.eye(3, dtype=np.float32)

    axis = axis / np.linalg.norm(axis)
    K = np.array([
        [0, -axis[2], axis[1]],
        [axis[2], 0, -axis[0]],
        [-axis[1], axis[0], 0],
    ], dtype=np.float32)

    R = np.eye(3, dtype=np.float32) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)
    
    return R

class NavFilter:

    def __init__(self, imu_model=None, twist_model=None, pose_model=None):
        # equivalente aos dois construtores C++ (default e o que recebe os 3 modelos)
        self.imu = imu_model
        self.twist = twist_model
        self.pose = pose_model

        self.error_state = ErrorState()
        self.state = State()

        # --- Membros usados em mechanization() mas não vistos no .hpp ---
        # TODO: confirmar valores/tipos reais contra nav_filter.hpp
        self.C_n_b = np.eye(3, dtype=np.float32)
        self.gravity = np.array([0.0, 0.0, 9.81], dtype=np.float32)  # confirmar sinal/convenção (NED)

        self.alfa = np.zeros(3, dtype=np.float32)
        self.beta = np.zeros(3, dtype=np.float32)
        self.last_gyro = np.zeros(3, dtype=np.float32)
        self.last_alfa = np.zeros(3, dtype=np.float32)
        self.last_beta = np.zeros(3, dtype=np.float32)
        self.last_alfa_delta = np.zeros(3, dtype=np.float32)
        self.last_orientation = np.zeros(3, dtype=np.float32)

    def propagate_error(self, imu_measurement: np.ndarray):
        A_k = self.imu.get_jacobian(self.error_state, imu_measurement)
        B_k = self.imu.get_noise_jacobian(self.error_state)
        P_old = self.error_state.error_covariance_

        # equivalente a: this->error_state = A_k * this->error_state.get_error_vector();
        self.error_state.set_from_vector(A_k @ self.error_state.get_error_vector())

        self.error_state.error_covariance_ = (
            A_k @ P_old @ A_k.T + B_k @ self.imu.covariance_ @ B_k.T
        )

    def update_ekf(self,
                    measurement: np.ndarray,
                    measurement_covariance: np.ndarray,
                    jacobian_matrix: np.ndarray,
                    predicted_measurement: np.ndarray,
                    noise_jacobian_matrix: np.ndarray):

        P_ = self.error_state.error_covariance_
        H_k1 = jacobian_matrix
        S_k1 = measurement_covariance
        M_k1 = noise_jacobian_matrix

        S = H_k1 @ P_ @ H_k1.T + M_k1 @ S_k1 @ M_k1.T
        K_k1 = P_ @ H_k1.T @ np.linalg.inv(S)

        error_z = measurement - predicted_measurement

        error_vector = self.error_state.get_error_vector()
        new_error_vector = error_vector + K_k1 @ (error_z - H_k1 @ error_vector)
        self.error_state.set_from_vector(new_error_vector)

        self.error_state.error_covariance_ = (
            np.eye(18, dtype=np.float32) - K_k1 @ H_k1
        ) @ P_

    def update_pose(self, pose_measurement: np.ndarray):
        residual = np.concatenate([
            self.state.position_ - pose_measurement[0:3],
            self.state.orientation_ - pose_measurement[3:6],
        ]).astype(np.float32)

        jacobian = self.pose.get_jacobian(self.error_state)
        predicted_measurement = self.pose.get_measurement(self.error_state)
        measurement_covariance = self.pose.get_covariance().astype(np.float32)
        noise_jacobian = self.pose.get_noise_jacobian(self.error_state)

        self.update_ekf(residual, measurement_covariance,
                        jacobian, predicted_measurement, noise_jacobian)

        self.state.position_ = self.state.position_ + self.error_state.error_position_

        # correção que já tínhamos adicionado antes:
        self.state.orientation_ = self.state.orientation_ + self.error_state.error_orientation_
        self.last_orientation = self.state.orientation_.copy()
        self.alfa = np.zeros(3, dtype=np.float32)
        self.beta = np.zeros(3, dtype=np.float32)
        self.last_alfa_delta = np.zeros(3, dtype=np.float32)
        self.last_alfa_delta = np.zeros(3, dtype=np.float32)

    def update_twist(self, twist_measurement: np.ndarray):
        jacobian = self.twist.get_jacobian(self.error_state, self.state)
        predicted_measurement = self.twist.get_measurement(self.error_state)
        measurement_covariance = self.twist.get_covariance().astype(np.float32)
        noise_jacobian = self.twist.get_noise_jacobian(self.error_state)

        self.update_ekf(twist_measurement, measurement_covariance,
                         jacobian, predicted_measurement, noise_jacobian)

        self.state.velocity_ = self.state.velocity_ + self.error_state.error_velocity_

    def update_imu(self, imu_measurement: np.ndarray):
        self.mechanization(imu_measurement)
        self.propagate_error(imu_measurement)

    def mechanization(self, imu_measurement: np.ndarray):
        accel = imu_measurement[0:3]   # .head<3>()
        gyro = imu_measurement[3:6]    # .tail<3>()

        velocity_dot = self.C_n_b @ accel + self.gravity

        dt = self.imu.update_time_

        alfa_delta = (gyro + self.last_gyro) / 2 * dt
        beta_delta = 0.5 * np.cross(self.alfa + (1.0 / 6.0) * self.last_alfa_delta, alfa_delta)

        self.alfa = self.alfa + alfa_delta
        self.beta = self.beta + beta_delta

        self.last_gyro = gyro
        self.last_alfa = self.alfa
        self.last_beta = self.beta
        self.last_alfa_delta = alfa_delta

        self.state.position_ = self.state.position_ + self.state.velocity_ * dt
        self.state.velocity_ = self.state.velocity_ + velocity_dot * dt
        self.state.orientation_ = self.alfa + self.beta + self.last_orientation

        angle = np.linalg.norm(self.state.orientation_)
        axis = (self.state.orientation_ / angle) if angle != 0 else np.array([1.0, 0.0, 0.0], dtype=np.float32)
        self.C_n_b = angle_axis_to_rotation_matrix(angle, axis)

    def get_state(self) -> State:
        return self.state