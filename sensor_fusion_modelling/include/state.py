import numpy as np


class State:

    def __init__(self, covariance: np.ndarray = None):
        self.position_ = np.zeros(3, dtype=np.float32)
        self.velocity_ = np.zeros(3, dtype=np.float32)
        self.orientation_ = np.zeros(3, dtype=np.float32)
        self.angular_velocity_ = np.zeros(3, dtype=np.float32)

        if covariance is not None:
            # equivalente a State(Eigen::Matrix<float,12,12> covariance)
            self.covariance_ = np.asarray(covariance, dtype=np.float32)
        else:
            # equivalente a State() default: identidade * 1e-8
            self.covariance_ = np.eye(12, dtype=np.float32) * 1e-8

    def get_covariance(self) -> np.ndarray:
        return self.covariance_

    def get_state_vector(self) -> np.ndarray:
        return np.concatenate([
            self.position_, self.velocity_,
            self.orientation_, self.angular_velocity_,
        ]).astype(np.float32)