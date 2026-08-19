import numpy as np


class SensorModel:
    """
    Equivalente Python de SensorModel (sensor_model.hpp/cpp).

    Estruturalmente quase idêntico a ProcessModel, com duas diferenças:
      - covariance_ fica em float64 (MatrixXd), sem cast pra float32
        (repare: get_covariance() no C++ original retorna Eigen::MatrixXd,
        não MatrixXf, diferente do ProccessModel::get_covariance())
      - tem um método virtual a mais: get_measurement()
    """

    def __init__(self, transform: np.ndarray = None,
                 noise: list = None,
                 dt: float = 0.0):
        if transform is None and noise is None:
            # equivalente ao construtor default SensorModel::SensorModel()
            self.transform_ = None
            self.update_time_ = 0.0
            self.covariance_ = None
            
            return

        # equivalente ao construtor SensorModel::SensorModel(transform, noise, dt)
        self.transform_ = np.asarray(transform, dtype=np.float32)
        self.update_time_ = float(dt)

        # note: aqui fica em float64, sem cast — diferente do ProccessModel
        noise_float = np.asarray(noise, dtype=np.float64)
        self.covariance_ = np.diag(noise_float)

    # --- getters ---

    def get_update_time(self) -> float:
        return self.update_time_

    def get_transform(self) -> np.ndarray:
        return self.transform_

    def get_covariance(self) -> np.ndarray:
        return self.covariance_

    # --- métodos "virtuais" com implementação default ---

    def get_jacobian(self, state) -> np.ndarray:
        return np.empty((0, 0), dtype=np.float32)

    def get_noise_jacobian(self, state) -> np.ndarray:
        return np.empty((0, 0), dtype=np.float32)

    def get_measurement(self, state) -> np.ndarray:
        return np.empty((0, 0), dtype=np.float32)