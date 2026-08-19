import numpy as np
from include.process_model import ProcessModel


class IMUModel(ProcessModel):

    def __init__(self,
                 transform_base: np.ndarray,      # (4,4) float32
                 acc_random_walk: list,
                 gyro_random_walk: list,
                 acc_noise: list,
                 gyro_noise: list,
                 corr_noise: list,
                 corr_transform: np.ndarray,      # (3,3) float64
                 dt: float):
        # Não chama super().__init__() com os parâmetros da base, pois a
        # covariância é montada de um jeito custom aqui (bloco a bloco).
        self.transform_ = np.asarray(transform_base, dtype=np.float32)
        self.update_time_ = float(dt)
        self.corr_transform_ = np.asarray(corr_transform, dtype=np.float64)

        eye_block = np.eye(3, dtype=np.float64) * 10e-4

        acc_noise_diag  = np.diag(np.asarray(acc_noise, dtype=np.float64))
        gyro_noise_diag = np.diag(np.asarray(gyro_noise, dtype=np.float64))
        acc_rw_diag     = np.diag(np.asarray(acc_random_walk, dtype=np.float64))
        gyro_rw_diag    = np.diag(np.asarray(gyro_random_walk, dtype=np.float64))
        corr_noise_diag = np.diag(np.asarray(corr_noise, dtype=np.float64))

        # Monta a matriz de covariância como bloco-diagonal (equivalente aos
        # vários .block(current_row, current_row, ...) sequenciais no C++)
        blocks = [eye_block, acc_noise_diag, gyro_noise_diag,
                  acc_rw_diag, gyro_rw_diag, corr_noise_diag]

        total_size = sum(b.shape[0] for b in blocks)
        covariance = np.zeros((total_size, total_size), dtype=np.float64)

        current = 0
        for b in blocks:
            n = b.shape[0]
            covariance[current:current + n, current:current + n] = b
            current += n

        self.covariance_ = covariance.astype(np.float32)

    def get_jacobian(self, state, control_vector: np.ndarray) -> np.ndarray:
        jacobian = np.zeros((18, 18), dtype=np.float32)
        dt = self.update_time_

        a1, a2, a3 = control_vector[0], control_vector[1], control_vector[2]
        v1, v2, v3 = state.error_velocity_[0], state.error_velocity_[1], state.error_velocity_[2]
        lambda1, lambda2, lambda3 = (state.error_orientation_[0],
                                      state.error_orientation_[1],
                                      state.error_orientation_[2])
        bw1, bw2, bw3 = state.error_bw_[0], state.error_bw_[1], state.error_bw_[2]

        t1 = self.corr_transform_[0, 0]
        t2 = self.corr_transform_[1, 1]
        t3 = self.corr_transform_[2, 2]

        jacobian[0, 0] = 1
        jacobian[0, 3] = dt
        jacobian[1, 1] = 1
        jacobian[1, 4] = dt
        jacobian[2, 2] = 1
        jacobian[2, 5] = dt

        jacobian[3, 3] = 1 - dt
        jacobian[3, 4] = -dt * lambda1
        jacobian[3, 5] = -dt * lambda2
        jacobian[3, 6] = -dt * (v2 - a1 * lambda3 + a2 * lambda2)
        jacobian[3, 7] = -dt * (a1 + v3 + a2 * lambda1 + 2 * a3 * lambda2)
        jacobian[3, 8] = -dt * (a2 - a1 * lambda1 + 2 * a3 * lambda3)

        jacobian[4, 3] = dt * lambda1
        jacobian[4, 4] = 1 - dt
        jacobian[4, 5] = -dt * lambda3
        jacobian[4, 6] = dt * (a1 + v1 + 2 * a2 * lambda1 + a3 * lambda2)
        jacobian[4, 7] = dt * (a1 * lambda3 + a3 * lambda1)
        jacobian[4, 8] = -dt * (a3 + v3 - a1 * lambda2 - 2 * a2 * lambda3)

        jacobian[5, 3] = dt * lambda2
        jacobian[5, 4] = dt * lambda3
        jacobian[5, 5] = 1 - dt
        jacobian[5, 6] = dt * (a2 - 2 * a1 * lambda1 + a3 * lambda3)
        jacobian[5, 7] = dt * (a3 + v1 - 2 * a1 * lambda2 - a2 * lambda3)
        jacobian[5, 8] = dt * (v2 - a2 * lambda2 + a3 * lambda1)

        jacobian[6, 6]  = 1 - bw2 * dt
        jacobian[6, 7]  = -bw3 * dt
        jacobian[6, 12] = -dt
        jacobian[6, 13] = -dt * lambda1
        jacobian[6, 14] = -dt * lambda2

        jacobian[7, 6]  = bw1 * dt
        jacobian[7, 7]  = 1
        jacobian[7, 8]  = -bw3 * dt
        jacobian[7, 12] = dt * lambda1
        jacobian[7, 13] = -dt
        jacobian[7, 14] = -dt * lambda3

        jacobian[8, 7]  = bw1 * dt
        jacobian[8, 8]  = bw2 * dt + 1
        jacobian[8, 12] = dt * lambda2
        jacobian[8, 13] = dt * lambda3
        jacobian[8, 14] = -dt

        jacobian[15, 15] = 1 - dt / t1
        jacobian[16, 16] = 1 - dt / t2
        jacobian[17, 17] = 1 - dt / t3

        return jacobian

    def get_noise_jacobian(self, state) -> np.ndarray:
        dt = self.update_time_
        lambda1, lambda2, lambda3 = (state.error_orientation_[0],
                                      state.error_orientation_[1],
                                      state.error_orientation_[2])

        jacobian = np.zeros((18, 18), dtype=np.float32)

        # --- Bloco 1: Posição (0,0 a 2,2) ---
        jacobian[0, 0] = dt + 1
        jacobian[1, 1] = dt + 1
        jacobian[2, 2] = dt + 1

        # --- Bloco 2: Velocidade + Skew-Symmetric (3,3 a 5,5) ---
        jacobian[3, 3] = dt + 1
        jacobian[3, 4] = dt * lambda1
        jacobian[3, 5] = dt * lambda2

        jacobian[4, 3] = -dt * lambda1
        jacobian[4, 4] = dt + 1
        jacobian[4, 5] = dt * lambda3

        jacobian[5, 3] = -dt * lambda2
        jacobian[5, 4] = -dt * lambda3
        jacobian[5, 5] = dt + 1

        # --- Bloco 3: Orientação + Skew-Symmetric (6,6 a 8,8) ---
        jacobian[6, 6] = dt + 1
        jacobian[6, 7] = dt * lambda1
        jacobian[6, 8] = dt * lambda2

        jacobian[7, 6] = -dt * lambda1
        jacobian[7, 7] = dt + 1
        jacobian[7, 8] = dt * lambda3

        jacobian[8, 6] = -dt * lambda2
        jacobian[8, 7] = -dt * lambda3
        jacobian[8, 8] = dt + 1

        # --- Bloco 4: Biases / Decaimento (9,9 a 14,14) ---
        jacobian[9, 9]   = 1 - dt
        jacobian[10, 10] = 1 - dt
        jacobian[11, 11] = 1 - dt
        jacobian[12, 12] = 1 - dt
        jacobian[13, 13] = 1 - dt
        jacobian[14, 14] = 1 - dt

        # --- Bloco 5: Estados Finais (15,15 a 17,17) ---
        jacobian[15, 15] = dt + 1
        jacobian[16, 16] = dt + 1
        jacobian[17, 17] = dt + 1

        return jacobian