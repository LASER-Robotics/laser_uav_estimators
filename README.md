# LASER UAV Estimators

This package provides **C++ classes** and **ROS 2 nodes** responsible for the state estimation of the Laser UAV System (LUS), translating sensor data into accurate odometry.

## Overview and Algorithms

The main objective of this package is to provide a robust estimation loop capable of fusing multiple sensor sources.

### State Estimator
-   **Description:** Implements an Extended Kalman Filter (EKF) that utilizes a dynamic model of the UAV for state prediction and corrects it using available sensor measurements. It handles data from diverse sources such as PX4 odometry, OpenVINS, FastLIO, and IMU data to compute the drone's position, orientation, and velocity. This node organizes the prediction and correction steps of the filter to provide a fused state estimate.

-   **Reference:**
    ```bibtex
    S. Thrun, W. Burgard, and D. Fox, Probabilistic Robotics. MIT Press, 2005.
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