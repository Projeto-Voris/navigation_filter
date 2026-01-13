# Navigation Filter
A Kalman Filter is a mathematical algorithm used to estimate the true state of a dynamic system (position, velocity, etc.) by combining noisy sensor measurements with a mathematical model. It minimizes the error between the estimated state and the ground truth by fusing data from various sensors and localization/mapping algorithms.

Within this package node, we utilize callbacks for Pose, Twist, and Odometry:

## Pose Callback
The Pose Callback processes and stores all spatial data received from the ORB-SLAM algorithm.

### Message Type

### ORBSLAM
ORB-SLAM is a versatile and accurate SLAM (Simultaneous Localization and Mapping) solution. In this context, it provides the vehicle's estimated position and orientation relative to a mapped environment using visual features.

## Twist Callback
The Twist Callback handles velocity data received from the DVL (Doppler Velocity Log).

### Message Type

### DVL

## Odom Callback
The Odom Callback processes orientation and acceleration data typically provided by the IMU (Inertial Measurement Unit).

### Message Type

### Imu
The IMU measures specific force and angular rate using a combination of accelerometers and gyroscopes. It provides high-frequency updates on the vehicle's attitude (roll, pitch, and yaw), serving as a backbone for the filter's prediction step.

# Commands and Compilation
To build and run the navigation filter within your workspace, use the following commands:
### 1. Build the Workspace

### 2. Source the Environment

### 3. Launch the Node
