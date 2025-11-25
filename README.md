# LASER UAV Estimators

This package provides **C++ classes** and **ROS 2 nodes** responsible for the state estimation of the Laser UAV System (LUS), translating sensor data into accurate odometry.

`laser_uav_estimator` is a ROS 2 package that provides a robust **13-state Extended Kalman Filter (EKF)** for Unmanned Aerial Vehicles (UAVs). Its primary function is to fuse data from multiple odometry sources (e.g., OpenVINS, Fast-LIO, PX4) and an Inertial Measurement Unit (IMU) to deliver a stable and reliable state estimate.

A key feature of this estimator is its use of **automatic differentiation** (via the `autodiff` library) to calculate the Jacobian matrices for the prediction step. This approach simplifies the implementation of the complex drone dynamics and significantly reduces the potential for mathematical errors in the linearization process.

The filter is also designed for resilience; it can handle the absence of any sensor measurement at a given timestep, making it robust against sensor dropouts.

## Key Features

* **13-State EKF**: Estimates the drone's full state, including position, orientation (quaternion), and linear and angular velocities.
* **Multi-Sensor Fusion**: Fuses various odometry sources, IMU data, and GPS in a single, batched correction step for efficiency.
* **Automatic Differentiation**: Avoids manual Jacobian calculation, leading to a more reliable and maintainable implementation.
* **Robustness**: Gracefully handles missing measurements from any sensor without interrupting the estimation process.
* **Highly Configurable**: All physical drone parameters and noise characteristics can be tuned via a single YAML file.
* **Modern C++ Library**: Designed as a C++ library for easy integration into other ROS 2 nodes.

## Overview and Algorithms

The main objective of this package is to provide a robust estimation loop capable of fusing multiple sensor sources.

### State Estimator
-   **Description:** Implements an Extended Kalman Filter (EKF) that utilizes a dynamic model of the UAV for state prediction and corrects it using available sensor measurements. It handles data from diverse sources such as PX4 odometry, OpenVINS, FastLIO, and IMU data to compute the drone's position, orientation, and velocity. This node organizes the prediction and correction steps of the filter to provide a fused state estimate.

-   **State Vector:**
    The filter estimates the following 13 states of the UAV:

    | Index | Enum      | Description                                                | Unit    |
    | :---- | :-------- | :--------------------------------------------------------- | :------ |
    | 0     | `PX`      | Position in the X-axis (inertial frame)                    | m       |
    | 1     | `PY`      | Position in the Y-axis (inertial frame)                    | m       |
    | 2     | `PZ`      | Position in the Z-axis (inertial frame)                    | m       |
    | 3     | `QW`      | W component (real) of the orientation quaternion           | -       |
    | 4     | `QX`      | X component (i) of the orientation quaternion              | -       |
    | 5     | `QY`      | Y component (j) of the orientation quaternion              | -       |
    | 6     | `QZ`      | Z component (k) of the orientation quaternion              | -       |
    | 7     | `VX`      | Linear velocity in the X-axis (BODY frame)                 | m/s     |
    | 8     | `VY`      | Linear velocity in the Y-axis (BODY frame)                 | m/s     |
    | 9     | `VZ`      | Linear velocity in the Z-axis (BODY frame)                 | m/s     |
    | 10    | `WX`      | Angular velocity in the X-axis (BODY frame) - Roll rate    | rad/s   |
    | 11    | `WY`      | Angular velocity in the Y-axis (BODY frame) - Pitch rate   | rad/s   |
    | 12    | `WZ`      | Angular velocity in the Z-axis (BODY frame) - Yaw rate     | rad/s   |


-   **Reference:**
    ```bibtex
    @book{thrun2005probabilistic,
      title={Probabilistic robotics},
      author={Thrun, Sebastian and Burgard, Wolfram and Fox, Dieter},
      year={2005},
      publisher={MIT press}
    }
    ```
-   **Configurable Parameters:**
    ```yaml
    state_estimator:
      ros__parameters:
        # Process Noise Gains (Q Matrix)
        # Determines the trust in the prediction model
        process_noise_gains:
          position: 0.1
          orientation: 0.01
          linear_velocity: 0.1
          angular_velocity: 0.1

        # Measurement Noise Gains (R Matrix)
        # Determines the uncertainty of each sensor
        measurement_noise_gains:
          px4_odometry:
            position: 0.1
            orientation: 0.1
            linear_velocity: 0.1
            angular_velocity: 0.1
            
          fast_lio:
            position: 0.001
            orientation: 0.001
            linear_velocity: 0.01
            angular_velocity: 0.01
            
          openvins:
            position: 0.01
            orientation: 0.01
            linear_velocity: 0.05
            angular_velocity: 0.05

          imu:
            angular_velocity: 0.05
            
    ```

## Sensor Interfaces

This section details the expected inputs and integration notes for the supported sensors.

### PX4 Odometry
-   **Description:** Odometry published by the PX4 flight controller (autopilot). It typically represents the best local estimate from the flight controller and may contain position, orientation, and velocities in the vehicle frame.
-   **ROS Topics:** `/$UAV_NAME/px4_api/odometry` (`nav_msgs/Odometry`).
-   **Recommended Frequency:** 100–200 Hz.
-   **Integration Notes:**
    -   Check the frames (`frame_id`, `child_frame_id`) and transform to the EKF estimation frame if necessary.
    -   Use PX4 covariances when available to adjust the R matrices.


### FastLIO Odometry
-   **Description:** Odometry produced by FastLIO (LiDAR). FastLIO provides pose and often relative velocities with good accuracy in structured environments.
-   **ROS Topics:** `/$UAV_NAME/fast_lio/odometry` (`nav_msgs/Odometry`).
-   **Recommended Frequency:** 5–20 Hz (dependent on LiDAR).
-   **Integration Notes:**
    -   LiDAR can provide robust estimates in lightless environments but may fail on reflective surfaces.
    -   Adjust covariance matrices depending on the scan-to-map matching rate.
    -   Combine with IMU to smooth high-frequency jitter and correct time offsets.
    -   Correct camera-to-body transformations (`tf`) before using measurements.

### OpenVINS Odometry
-   **Description:** Visual odometry produced by OpenVINS. Provides pose estimates that can complement IMU and LiDAR, yielding excellent results in textured environments.
-   **ROS Topics:** `/$UAV_NAME/openvins/odometry` (`nav_msgs/Odometry`).
-   **Recommended Frequency:** 100–200 Hz (dependent on camera pipeline).
-   **Integration Notes:**
    -   Synchronize timestamps between IMU and OpenVINS for correct fusion.
    -   Correct camera-to-body transformations (`tf`) before using measurements.
    -   Visual odometry may lose tracking in low-texture areas; handle failures as missing measurements.

### IMU Propagation
-   **Description:** IMU-based propagation used for the filter correction step (high rate, low short-term drift). The IMU provides linear accelerations and angular velocities which must be integrated and bias-compensated.
-   **ROS Topics:** `/$UAV_NAME/px4_api/imu` (`sensor_msgs/Imu`).
-   **Recommended Frequency:** 200–500 Hz.
-   **Integration Notes:**
    -   IMU must be integrated at the correct time; use `sensor_msgs/Imu` timestamps.
    -   Estimate and correct gyroscope and accelerometer biases.
