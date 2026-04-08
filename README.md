# Navigation Filter
A Kalman Filter is a mathematical algorithm used to estimate the true state of a dynamic system (position, velocity, etc.) by combining noisy sensor measurements with a mathematical model. It minimizes the error between the estimated state and the ground truth by fusing data from various sensors and localization/mapping algorithms.

## Pose Callback
The Pose Callback processes and stores all spatial data received from the ORB-SLAM algorithm.
### Message Type
| Module           | Direction | Topic                     | Message Type                 | Notes |
|------------------|-----------|---------------------------|-------------------------------|-------|
| Pose             | Sub       | `/pose`                   | `geometry_msgs/PoseStamped`     | Provides absolute position (x,y,z) and orientation (quaternions) to correct long-term drift. |
### ORBSLAM
ORB-SLAM is a versatile and accurate SLAM (Simultaneous Localization and Mapping) solution. In this context, it provides the vehicle's estimated position and orientation relative to a mapped environment using visual features.

## Twist Callback
The `Twist` Callback handles velocity data received from the DVL (Doppler Velocity Log).
### Message Type
| Module           | Direction | Topic                     | Message Type                 | Notes |
|------------------|-----------|---------------------------|-------------------------------|-------|
| Twist            | Sub       | `/twist`                  | `geometry_msgs/TwistStamped`     | Provides linear velocity data relative to the sea floor, which is critical for maintaining stability when visual features are lost. |
### DVL
The DVL is an acoustic sensor that measures the velocity of the vehicle relative to the sea floor. By calculating the Doppler shift of reflected sound waves, it provides precise linear velocity data, which is crucial for reducing drift in underwater navigation.  

## Odom Callback
The `Odom` Callback processes orientation and acceleration data typically provided by the IMU (Inertial Measurement Unit).
### Message Type
| Module           | Direction | Topic                     | Message Type                  | Notes |
|------------------|-----------|---------------------------|-------------------------------|-------|
| Odom             | Sub       | `/mavros/imu/data_raw`    | `sensor_msgs/Imu`            | Supplies angular velocity and linear acceleration, allowing the filter to "predict" movement between SLAM and DVL updates. |
### Imu
The `IMU` measures specific force and angular rate using a combination of accelerometers and gyroscopes. It provides high-frequency updates on the vehicle's attitude (roll, pitch, and yaw), serving as a backbone for the filter's prediction step.

## Publisher
Combined with the other packages (ORBSLAM and the DVL), this algoritm provides a Odometry msgs.
| Module           | Direction | Topic                     | Message Type                  | Notes |
|------------------|-----------|---------------------------|-------------------------------|-------|
| Odom             | Pub       | `/odom`                   | `nav_msgs/Odometry`           | Final Output |


# Commands and Compilation
To build and run the navigation filter within your workspace, use the following commands:
### 1. Build the Workspace
Using `colcon` (ROS 2) to compile the specific navigation package:
```bash
    colcon build --packages-select <package_name> 
   ```
### 2. Source the Environment
You must source the workspace in every new terminal to allow ROS to find your nodes:
```bash
    source install/setup.bash
   ```
### 3. Launch/Run the Node
```bash
    ros2 run navigation_filter nav_filter_node.cpp
   ```
```bash
    ros2 launch navigation_filter nav_filter_test.launch.py
   ```