# LASER UAV Estimators

`laser_uav_estimators` provides the C++ state estimators used by the LASER UAV System (LUS). It combines motion prediction with odometry measurements to estimate position, orientation and velocity for the control loop.

The `estimation_manager` node in `laser_uav_managers` selects the odometry source, buffers measurements and calls the estimator. This package contains the filter classes and tuning files; it has no standalone ROS node.

## Estimators

| Class | Prediction input | Correction input | Role |
| --- | --- | --- | --- |
| `MEKFEstimator` | Motor control inputs and timestep, using mass, inertia and the allocation matrix. | Selected odometry and optional range measurements. | Multiplicative error-state filter used by the supplied estimation-manager launch. |
| `ESKFEstimator` | IMU acceleration, angular velocity and timestep. | PX4, FAST-LIO and OpenVINS odometry. | Error-state filter with IMU biases, gravity estimation and odometry innovation limits. |

Both classes expose `predict()`, `correct()` and getters for the estimated state. `get_odometry()` returns a `nav_msgs/msg/Odometry`; the calling node publishes it.

## Configuration

[params/](params/) contains separate tuning files for the two filters and their odometry sources.

| Files | Used for |
| --- | --- |
| `mekf_px4_api_state_estimator.yaml` | MEKF with autopilot odometry. |
| `mekf_fast_lio_state_estimator.yaml` | MEKF with FAST-LIO odometry. |
| `mekf_openvins_state_estimator .yaml` | MEKF with OpenVINS. The filename currently contains a space before `.yaml`. |
| `eskf_px4_state_estimator.yaml` | ESKF tuning for autopilot odometry. |
| `eskf_lio_state_estimator.yaml` | ESKF tuning for LiDAR odometry. |
| `eskf_vio_state_estimator.yaml` | ESKF tuning for visual odometry. |

The OpenVINS MEKF file has a different name from the one requested by the estimation-manager launch. Check the loaded file when selecting `openvins_odom`; that launch expects `mekf_openvins_state_estimator.yaml` without the space.

### MEKF parameters

| Parameter group | Meaning |
| --- | --- |
| `process_noise_gains.position_xy`, `position_z` | Prediction uncertainty for horizontal and vertical position. |
| `process_noise_gains.orientation` | Prediction uncertainty for attitude. |
| `process_noise_gains.linear_velocity_xy`, `linear_velocity_z`, `angular_velocity` | Prediction uncertainty for linear and angular velocity. |
| `measurement_noise_gains.<source>.*` | Measurement uncertainty for `px4_odometry`, `fast_lio` or `openvins`, using the same state groups. |
| `measurement_noise_gains.garmin.position_z` | Uncertainty assigned to range-based height measurements. |

### ESKF parameters

| Parameter group | Meaning |
| --- | --- |
| `process_noise_gains.velocity`, `orientation` | Velocity and attitude process-noise gains. |
| `process_noise_gains.accelerometer`, `gyroscope` | Accelerometer and gyroscope bias process-noise gains. |
| `measurement_noise_gains.<source>.*` | Uncertainty assigned to the measurements from each configured source. |
| `limits.<source>.position.{x,y,z}` | Position innovation limits in metres. |
| `limits.<source>.orientation.{roll,pitch,yaw}` | Attitude innovation limits in radians. |
| `ekf_verbosity` | Logging level: `SILENT`, `ERROR`, `WARNING`, `INFO`, `DEBUG` or `VERBOSE`. |

Larger measurement-noise values reduce the weight of that measurement relative to prediction. Tune against the sensor's noise and update rate. Choosing an ESKF YAML alone does not switch the manager from MEKF to ESKF; the caller chooses the filter class.

## Build and use

From the workspace root:

```bash
colcon build --packages-up-to laser_uav_estimators
source install/setup.bash
```

For normal LUS operation, configure and launch `estimation_manager`. Its README documents the ROS topics, odometry-source selection and launch arguments.

## Reference

S. Thrun, W. Burgard and D. Fox, *Probabilistic Robotics*, MIT Press, 2005.
