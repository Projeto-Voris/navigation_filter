import numpy as np 

class ErroSpaceKalmanFilter:
    def __init__():
        erro_x = np.zeros(18, 1)
        covariance_erro_x = np.eye(18)*(1e-8)
        imu_counter = 0
        dvl_counter = 0
        slam_counter = 0
        start_time = 0
        slam_covariance = np.eye(15)
        dvl_covariance = np.eye(15)
        velocity = [0; 0; 0]
        position = [0; 0; 0]
        orientation = [0;0;0]
        last_integration = [0;0;0]
        last_orientation = [0;0;0]

        earth_radius = 6.3781e6
        earth_rate = 7.2921150e-5
        gravity = [0; 0; 9.81]
        C_n_b = np.eye(3)
        C_n_e = np.eye(3)

        alfa = [0; 0; 0]
        beta = [0; 0; 0]
        last_alfa = [0; 0; 0]
        last_beta = [0; 0; 0]
        last_alfa_delta = [0; 0; 0]
        last_beta_delta = [0; 0; 0]
        last_gyro = [0; 0; 0]

        proccess_covariance = np.eye(18)
        error_position = np.zeros(3, 1)
        error_velocity = np.zeros(3, 1)
        error_orientation = np.zeros(3, 1)



