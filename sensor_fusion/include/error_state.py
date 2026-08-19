import numpy as np


class ErrorState:
    """
    Equivalente Python de ErrorState (error_state.hpp/cpp).

    Mapeamento de tipos:
      Eigen::Vector<float, 3>        -> np.ndarray shape (3,)  dtype=float32
      Eigen::Matrix<float, 18, 18>   -> np.ndarray shape (18,18) dtype=float32
      Eigen::Vector<float, 18>       -> np.ndarray shape (18,)  dtype=float32
      operator=(vector18)            -> método set_from_vector() / __set_state__
      destrutor (~ErrorState)        -> não existe em Python (garbage collector cuida disso)
    """

    def __init__(self, error_covariance: np.ndarray = None):
        # equivalente aos dois construtores C++ (default e o que recebe a covariância)
        self.error_position_ = np.zeros(3, dtype=np.float32)
        self.error_velocity_ = np.zeros(3, dtype=np.float32)
        self.error_orientation_ = np.zeros(3, dtype=np.float32)

        self.error_ba_ = np.zeros(3, dtype=np.float32)
        self.error_bw_ = np.zeros(3, dtype=np.float32)
        self.bv_ = np.zeros(3, dtype=np.float32)

        if error_covariance is not None:
            error_covariance = np.asarray(error_covariance, dtype=np.float32)
            if error_covariance.shape != (18, 18):
                raise ValueError(
                    f"error_covariance deve ter shape (18, 18), recebeu {error_covariance.shape}"
                )
            self.error_covariance_ = error_covariance
        else:
            self.error_covariance_ = np.zeros((18, 18), dtype=np.float32)

    def set_from_vector(self, error_vector: np.ndarray) -> "ErrorState":
        """
        Equivalente ao operator=(const Eigen::Vector<float, 18>& error_vector).
        Em Python não dá pra sobrecarregar '=', então usamos um método explícito.
        Uso: state.set_from_vector(v)   (em vez de "state = v" no C++)
        """
        error_vector = np.asarray(error_vector, dtype=np.float32).flatten()
        if error_vector.shape[0] != 18:
            raise ValueError(
                f"error_vector deve ter tamanho 18, recebeu {error_vector.shape[0]}"
            )

        self.error_position_    = error_vector[0:3]
        self.error_velocity_    = error_vector[3:6]
        self.error_orientation_ = error_vector[6:9]
        self.error_ba_          = error_vector[9:12]
        self.error_bw_          = error_vector[12:15]
        self.bv_                = error_vector[15:18]

        return self

    def get_covariance(self) -> np.ndarray:
        return self.error_covariance_

    def get_error_vector(self) -> np.ndarray:
        """Concatena os 6 blocos de volta em um único vetor de tamanho 18."""
        return np.concatenate([
            self.error_position_,
            self.error_velocity_,
            self.error_orientation_,
            self.error_ba_,
            self.error_bw_,
            self.bv_,
        ]).astype(np.float32)