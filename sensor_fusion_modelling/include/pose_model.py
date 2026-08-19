import numpy as np
from process_model import ProcessModel


class PoseModel(ProcessModel):

    def get_jacobian(self, state) -> np.ndarray:
        jacobian = np.zeros((6, 18), dtype=np.float32)
        jacobian[0, 0] = -1
        jacobian[1, 1] = -1
        jacobian[2, 2] = -1
        jacobian[3, 6] = -1
        jacobian[4, 7] = -1
        jacobian[5, 8] = -1

        return jacobian

    def get_noise_jacobian(self, state) -> np.ndarray:
        jacobian = np.zeros((6, 6), dtype=np.float32)
        jacobian[0, 0] = -1
        jacobian[1, 1] = -1
        jacobian[2, 2] = -1
        # BUG? no C++ original: jacobian(3,6), (4,7), (5,8) -- índices 6,7,8
        # não existem numa matriz 6x6 (só vai de 0 a 5). Traduzido fiel abaixo
        # pra manter o mesmo comportamento (o numpy VAI estourar IndexError
        # aqui, diferente do Eigen em C++ que só corrompe memória
        # silenciosamente). Confirma isso antes de rodar.
        jacobian[3, 6] = -1
        jacobian[4, 7] = -1
        jacobian[5, 8] = -1

        return jacobian

    def get_measurement(self, state) -> np.ndarray:
        error_state = state.get_error_vector()          # shape (18,)
        error_position = error_state[0:3]                # segment(0,3)
        # BUG? no C++ original: segment(7,3) pega índices 7,8,9 -- mas pela
        # estrutura do ErrorState a orientação é [6:9], não [7:10].
        # Traduzido fiel abaixo:
        error_orientation = error_state[7:10]             # segment(7,3)

        result = np.concatenate([error_position, error_orientation]).astype(np.float32)
        
        return -result