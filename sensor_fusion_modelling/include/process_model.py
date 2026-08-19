import numpy as np


class ProcessModel:
    """
    Equivalente Python de ProccessModel (proccess_model.hpp/cpp).

    Mapeamento de tipos:
      Eigen::Matrix4f            -> np.ndarray shape (4,4)  dtype=float32
      Eigen::MatrixXf            -> np.ndarray shape (N,M)  dtype=float32
      Eigen::VectorXf            -> np.ndarray shape (N,)   dtype=float32
      std::vector<double> noise  -> list[float] (convertido p/ float32 igual ao C++)
      construtor de cópia        -> não precisa: objetos Python já usam referência;
                                     se quiser cópia de verdade, usar copy.deepcopy()
    """

    def __init__(self, transform: np.ndarray = None,
                 noise: list = None,
                 dt: float = 0.0):
        if transform is None and noise is None:
            # equivalente ao construtor default ProccessModel::ProccessModel()
            self.transform_ = None
            self.update_time_ = 0.0
            self.covariance_ = None
            return

        # equivalente ao construtor ProccessModel::ProccessModel(transform, noise, dt)
        self.transform_ = np.asarray(transform, dtype=np.float32)
        self.update_time_ = float(dt)

        noise_float = np.asarray(noise, dtype=np.float32)
        # equivalente ao Eigen::Map<VectorXf>(...).asDiagonal()
        self.covariance_ = np.diag(noise_float).astype(np.float32)

    # --- getters ---

    def get_update_time(self) -> float:
        return self.update_time_

    def get_transform(self) -> np.ndarray:
        return self.transform_

    def get_covariance(self) -> np.ndarray:
        return self.covariance_

    # --- métodos "virtuais" (para override nas subclasses) ---

    def get_jacobian(self, state, control_vector: np.ndarray) -> np.ndarray:
        """Implementação padrão (equivalente ao retorno de Eigen::MatrixXf() vazio)."""
        return np.empty((0, 0), dtype=np.float32)

    def get_noise_jacobian(self, state) -> np.ndarray:
        """Implementação padrão (equivalente ao retorno de Eigen::MatrixXf() vazio)."""
        return np.empty((0, 0), dtype=np.float32)