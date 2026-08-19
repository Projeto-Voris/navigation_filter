import numpy as np
from process_model import ProcessModel  # ou de onde você importar a classe base


class TwistModel(ProcessModel):

    def get_jacobian(self, e_state, state) -> np.ndarray:
        lambda1 = e_state.error_orientation_[0]
        lambda2 = e_state.error_orientation_[1]
        lambda3 = e_state.error_orientation_[2]

        bv1 = e_state.bv_[0]
        bv2 = e_state.bv_[1]
        bv3 = e_state.bv_[2]

        v_nom1 = state.velocity_[0]
        v_nom2 = state.velocity_[1]
        v_nom3 = state.velocity_[2]

        jacobian = np.zeros((6, 18), dtype=np.float32)

        # Velocidade linear (linhas 0-2)
        jacobian[0, 3]  = -1
        jacobian[0, 6]  = bv2
        jacobian[0, 7]  = bv3 - v_nom1
        jacobian[0, 8]  = -v_nom2
        jacobian[0, 15] = 1
        jacobian[0, 16] = lambda1
        jacobian[0, 17] = lambda2

        jacobian[1, 4]  = -1
        jacobian[1, 6]  = v_nom1 - bv1
        jacobian[1, 8]  = bv3 - v_nom3
        jacobian[1, 15] = -lambda1
        jacobian[1, 16] = 1
        jacobian[1, 17] = lambda3

        jacobian[2, 5]  = -1
        jacobian[2, 6]  = v_nom2
        jacobian[2, 7]  = v_nom3 - bv1
        jacobian[2, 8]  = -bv2
        jacobian[2, 15] = -lambda2
        jacobian[2, 16] = -lambda3
        jacobian[2, 17] = 1

        # Velocidade angular (linhas 3-5): zeradas, sem observabilidade
        # (já ficam zero por causa do np.zeros)

        return jacobian

    def get_noise_jacobian(self, e_state) -> np.ndarray:
        lambda1 = e_state.error_orientation_[0]
        lambda2 = e_state.error_orientation_[1]
        lambda3 = e_state.error_orientation_[2]

        jacobian = np.zeros((6, 6), dtype=np.float32)

        # Bloco linear (0:3, 0:3)
        jacobian[0:3, 0:3] = np.array([
            [1,        lambda1,  lambda2],
            [-lambda1, 1,        lambda3],
            [-lambda2, -lambda3, 1],
        ], dtype=np.float32)

        # Bloco angular (3:6, 3:6): identidade
        jacobian[3:6, 3:6] = np.eye(3, dtype=np.float32)

        return jacobian

    def get_measurement(self, state) -> np.ndarray:
        error_state = state.get_error_vector()          # shape (18,)
        error_velocity = error_state[3:6]
        error_orientation = error_state[6:9]

        result = np.concatenate([error_velocity, error_orientation]).astype(np.float32)  # shape (6,)
        return -result