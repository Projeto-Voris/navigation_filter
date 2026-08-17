import numpy as np 

def eul2rotm(euler_angles):
    """
    Converte ângulos de Euler [Roll (phi), Pitch (theta), Yaw (psi)] em radianos 
    para uma matriz de rotação 3x3 utilizando a ordem padrão do MATLAB (Z * Y * X).
    """
    phi = euler_angles[0]
    theta = euler_angles[1]
    psi = euler_angles[2]
    
    R_x = np.array([,
        [0, np.cos(phi), -np.sin(phi)],
        [0, np.sin(phi), np.cos(phi)]
    ])
                    
    R_y = np.array([
        [np.cos(theta), 0, np.sin(theta)],
,
        [-np.sin(theta), 0, np.cos(theta)]
    ])
                    
    R_z = np.array([
        [np.cos(psi), -np.sin(psi), 0],
        [np.sin(psi), np.cos(psi), 0],
        [0, 0, 1]
    ])
    
    return R_z @ R_y @ R_x

class ErrorSpaceKalmanFilter:
    def __init__(self, imu_update_time=0.01, dvl_update_time=0.1, slam_update_time=0.1,
                 accelerometer_random_walk_bias=None, gyroscope_random_walk_bias=None,
                 accelerometer_noise=None, gyroscope_noise=None, corr_noise=None,
                 dvl_noise=None, slam_noise=None, corr_t=None):
        
        # Propriedades Não-Ajustáveis (Nontunable)
        self.imu_update_time = imu_update_time
        self.dvl_update_time = dvl_update_time
        self.slam_update_time = slam_update_time
        
        self.accelerometer_random_walk_bias = np.array(accelerometer_random_walk_bias if accelerometer_random_walk_bias is not None else [1e-6, 1e-6, 1e-6]).reshape(3, 1)
        self.gyroscope_random_walk_bias = np.array(gyroscope_random_walk_bias if gyroscope_random_walk_bias is not None else [1e-6, 1e-6, 1e-6]).reshape(3, 1)
        self.accelerometer_noise = np.array(accelerometer_noise if accelerometer_noise is not None else [1e-6, 1e-6, 1e-6]).reshape(3, 1)
        self.gyroscope_noise = np.array(gyroscope_noise if gyroscope_noise is not None else [1e-6, 1e-6, 1e-6]).reshape(3, 1)
        self.corr_noise = np.array(corr_noise if corr_noise is not None else [1e-6, 1e-6, 1e-6]).reshape(3, 1)
        self.dvl_noise = np.array(dvl_noise if dvl_noise is not None else [1e-6, 1e-6, 1e-6]).reshape(3, 1)
        self.slam_noise = np.array(slam_noise if slam_noise is not None else [1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6]).reshape(6, 1)
        self.corr_t = np.array(corr_t if corr_t is not None else [1.0, 1.0, 1.0]).reshape(3, 1)

        # Propriedades Privadas (Private)
        self.error_x = np.zeros((18, 1))
        self.covariance_error_x = np.eye(18) * 1e-8

        self.imu_counter = 0
        self.dvl_counter = 0
        self.slam_counter = 0

        self.velocity = np.zeros((3, 1))
        self.position = np.zeros((3, 1))
        self.orientation = np.zeros((3, 1))
        self.last_integration = np.zeros((3, 1))
        self.last_orientation = np.zeros((3, 1))

        self.earth_radius = 6.3781e6
        self.earth_rate = 7.2921150e-5
        self.gravity = np.array([[0.0], [0.0], [9.81]])

        self.C_n_b = np.eye(3)
        self.C_n_e = np.eye(3)

        self.alfa = np.zeros((3, 1))
        self.beta = np.zeros((3, 1))
        self.last_alfa = np.zeros((3, 1))
        self.last_beta = np.zeros((3, 1))
        self.last_alfa_delta = np.zeros((3, 1))
        self.last_beta_delta = np.zeros((3, 1))
        self.last_gyro = np.zeros((3, 1))

        self.error_position = np.zeros((3, 1))
        self.error_velocity = np.zeros((3, 1))
        self.error_orientation = np.zeros((3, 1))
        
        self._setup_impl()


    def _setup_impl(self):
        self.slam_covariance = np.diag((self.slam_noise / self.slam_update_time).flatten())
        self.dvl_covariance = np.diag((self.dvl_noise / self.dvl_update_time).flatten())
        
        self.proccess_covariance = self._scipy_blkdiag(
            np.eye(3) * 10e-4,
            np.diag(self.accelerometer_noise.flatten()),
            np.diag(self.gyroscope_noise.flatten()),
            np.diag(self.accelerometer_random_walk_bias.flatten()),
            np.diag(self.gyroscope_random_walk_bias.flatten()),
            np.diag(self.corr_noise.flatten())
        )

    def step(self, imu, dvl, slam):
        imu = np.array(imu).reshape(-1, 1)
        dvl = np.array(dvl).reshape(-1, 1)
        slam = np.array(slam).reshape(-1, 1)

        pos, ori, vel = self.mechanization(imu[0:3], imu[3:6])

        self.update_imu(imu)
        self.slam_counter += 1
        self.dvl_counter += 1

        self.orientation = ori
        self.position = pos
        self.velocity = vel

        if self.dvl_counter >= (self.dvl_update_time / self.imu_update_time):
            self.update_dvl(dvl)
            self.dvl_counter = 0
            self.error_velocity = self.error_x[3:6]
            self.velocity = self.velocity + self.error_velocity

        if self.slam_counter >= (self.slam_update_time / self.imu_update_time):
            self.update_slam(slam)
            self.slam_counter = 0
            self.error_position = self.error_x[0:3]
            self.error_orientation = self.error_x[6:9]
            
            self.position = self.position + self.error_position

            self.alfa = np.zeros((3, 1))
            self.beta = np.zeros((3, 1))
            self.last_alfa_delta = np.zeros((3, 1))
            self.last_orientation = ori + self.error_orientation

        return self.position, self.velocity, self.orientation, self.error_x

    def update_imu(self, imu):
        A_k = self.getProccessJacobianMatrix(self.error_x, imu, self.imu_update_time, self.corr_t)
        self.error_x = A_k @ self.error_x
        B_k = self.getProccessNoiseJacobianMatrix(self.error_x, self.imu_update_time)
        self.covariance_error_x = A_k @ self.covariance_error_x @ A_k.T + B_k @ self.proccess_covariance @ B_k.T

    def update_dvl(self, dvl):
        R = self.C_n_b
        predicted_measurement = self.dvl_measurement(self.error_x, self.velocity, R)
        dvl_measurement = self.velocity - R @ dvl

        self.update_ekf(
            dvl_measurement, 
            self.dvl_covariance, 
            self.getDVLJacobian(self.error_x, self.velocity), 
            predicted_measurement, 
            self.getDVLNoiseJacobian(self.error_x)
        )

    def update_slam(self, slam):
        predicted_measurement = self.slam_measurement(self.error_x)
        slam_measurement = np.vstack([
            self.position - slam[0:3],
            self.orientation - slam[3:6]
        ])
        self.update_ekf(slam_measurement, self.slam_covariance, self.getSlamJacobian(self.error_x), predicted_measurement, np.eye(6))

    def update_ekf(self, measurement, measurement_covariance, jacobian_matrix, predicted_measurement, noise_jacobian_matrix):
        P_ = self.covariance_error_x
        H_k1 = jacobian_matrix
        S_k1 = measurement_covariance
        M_k1 = noise_jacobian_matrix
       
        innov_covariance = (H_k1 @ P_ @ H_k1.T) + (M_k1 @ S_k1 @ M_k1.T)
        K_k1 = P_ @ H_k1.T @ np.linalg.inv(innov_covariance)
        
        error_z = measurement - predicted_measurement

        self.error_x = self.error_x + K_k1 @ (error_z - H_k1 @ self.error_x)
        self.covariance_error_x = (np.eye(18) - K_k1 @ H_k1) @ P_

    def mechanization(self, Accel, Gyro):
        if Accel.shape != (3, 1) or Gyro.shape != (3, 1):
            raise ValueError("A entrada deve ser um vetor 3x1.")
      
        velocity_dot = self.C_n_b @ Accel + self.gravity
        
        alfa_delta = ((Gyro + self.last_gyro) / 2.0) * self.imu_update_time
        self.last_gyro = Gyro

        cross_term = np.cross(
            (self.alfa + (1.0/6.0) * self.last_alfa_delta).flatten(), 
            alfa_delta.flatten()
        ).reshape(3, 1)
        beta_delta = 0.5 * cross_term
        
        self.alfa = self.alfa + alfa_delta
        self.beta = self.beta + beta_delta

        self.last_alfa = self.alfa
        self.last_beta = self.beta
        self.last_alfa_delta = alfa_delta

        self.velocity = self.velocity + velocity_dot * self.imu_update_time
        self.position = self.position + self.velocity * self.imu_update_time

        Position = self.position
        Velocity = self.velocity
        Orientation = self.alfa + self.beta + self.last_orientation

        self.C_n_b = eul2rotm(Orientation.flatten())
        
        return Position, Orientation, Velocity