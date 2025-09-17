# Laser UAV Estimator

## Overview

`laser_uav_estimator` is a ROS 2 package that provides a robust 13-state Extended Kalman Filter (EKF) for Unmanned Aerial Vehicles (UAVs). Its primary function is to fuse data from multiple odometry sources (e.g., OpenVINS, Fast-LIO, PX4) and an Inertial Measurement Unit (IMU) to deliver a stable and reliable state estimate.

A key feature of this estimator is its use of **automatic differentiation** (via the `autodiff` library) to calculate the Jacobian matrices for the prediction step. This approach simplifies the implementation of the complex drone dynamics and significantly reduces the potential for mathematical errors in the linearization process.

The filter is also designed for resilience; it can handle the absence of any sensor measurement at a given timestep, making it robust against sensor dropouts.

## Key Features

  * **13-State EKF**: Estimates the drone's full state, including position, orientation (quaternion), and linear and angular velocities.
  * **Multi-Sensor Fusion**: Fuses various odometry sources, IMU data, and GPS in a single, batched correction step for efficiency.
  * **Automatic Differentiation**: Avoids manual Jacobian calculation, leading to a more reliable and maintainable implementation.
  * **Robustness**: Gracefully handles missing measurements from any sensor without interrupting the estimation process.
  * **Highly Configurable**: All physical drone parameters and noise characteristics can be tuned via a single YAML file.
  * **Modern C++ Library**: Designed as a C++ library for easy integration into other ROS 2 nodes.

## State Vector

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

## Dependencies

This package requires the following dependencies:

  * **ROS 2 Humble**
  * `ament_cmake`
  * `rclcpp`
  * `geometry_msgs`, `nav_msgs`, `sensor_msgs`
  * `laser_uav_lib`
  * **Eigen3**: For linear algebra operations.
    ```bash
    sudo apt-get update
    sudo apt-get install libeigen3-dev
    ```
  * **autodiff**: For automatic differentiation.

    ```sh
    cd /tmp
    echo "set -e
    mkdir -p ~/git
    cd ~/git/
    git clone https://github.com/autodiff/autodiff.git
    cd autodiff
    mkdir -p build && cd build
    cmake .. -DAUTODIFF_BUILD_PYTHON=OFF
    cmake --build . -- -j
    sudo cmake --install .
    echo 'Biblioteca autodiff compilada e instalada com sucesso!'" > run.sh && source run.sh
    ```
## Build Instructions

To build the package, clone it into your ROS 2 workspace and compile using `colcon`:

```bash
cd ~/git/
git clone git@github.com:LASail wagnergarcia@eng.ci.ufpb.br
 * @date September 16, 2025
 * @copyright Apache License 2.0
 * 
 * @details This package.xml file defines the configuration for the laser_uav_estimatorER-Robotics/laser_uav_estimator.git
cd ~/laser_uav_system_ws/src
ln -sf ~/git/laser_uav_estimator
cd ..
colcon build --packages-select laser_uav_estimator
```

## Configuration

All parameters for the EKF are managed in the `params/state_estimator.yaml` file. This includes the drone's physical properties and the noise gains for the filter.

### Drone Physical Parameters

These parameters define the dynamic model of the drone. It is crucial to set them accurately.

```yaml
drone_params:
  mass: 1.60                  # Total drone mass in kg
  arm_length: 0.258           # Not directly used, but good for reference
  thrust_coefficient: 1.0     # Motor thrust coefficient
  torque_coefficient: 0.059   # Motor torque coefficient
  inertia: [Ixx, Iyy, Izz]    # Inertia Matrix [kg*m^2]
  motor_positions: [...]      # (x, y) coordinates of each motor
```

### Process Noise Gains (`Q`)

These gains control the filter's trust in its own prediction model. Higher values mean less trust (uncertainty grows faster).

```yaml
process_noise_gains:
  position: 0.001
  orientation: 0.01
  linear_velocity: 0.01
  angular_velocity: 0.1
```

### Measurement Noise Gains (`R`)

These gains control how much the filter trusts each incoming sensor measurement. Lower values mean more trust (the filter will correct its state more aggressively based on the sensor data).

```yaml
measurement_noise_gains:
  px4_odometry:
    position: 0.0001
    orientation: 0.001
    linear_velocity: 0.01
    angular_velocity: 0.1
    # ... and so on for each sensor type
```

## Library Integration & Usage

This package is built as a shared library (`liblaser_uav_estimator.so`) intended to be used within another ROS 2 node. Here is a basic example of how to integrate it.

### 1\. Add Dependency

In your own package, add `laser_uav_estimator` as a dependency in your `package.xml` and `CMakeLists.txt`.

**`package.xml`:**

```xml
<depend>laser_uav_estimator</depend>
```

**`CMakeLists.txt`:**

```cmake
find_package(laser_uav_estimator REQUIRED)

# ...

ament_target_dependencies(your_node_name rclcpp laser_uav_estimator)
```

### 2\. C++ Implementation Example

Here is a simplified example of a ROS 2 node that uses the `StateEstimator`.

```cpp
#include <rclcpp/rclcpp.hpp>
#include <laser_uav_estimator/state_estimator.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

// Assuming you have callbacks for your sensor data
// void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
// void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

class MyDroneNode : public rclcpp::Node
{
public:
    MyDroneNode() : Node("my_drone_node")
    {
        // 1. Load drone parameters from your YAML file
        double mass = 1.60;
        double thrust_coeff = 1.0;
        double torque_coeff = 0.059;
        Eigen::Matrix3d inertia;
        inertia.setIdentity(); // Replace with real values
        Eigen::Matrix<double, 4, 2> motor_positions;
        motor_positions.setZero(); // Replace with real values

        // 2. Instantiate the estimator
        estimator_ = std::make_unique<laser_uav_estimator::StateEstimator>(
            mass, motor_positions, thrust_coeff, torque_coeff, inertia, "INFO");

        // Assume you load and set noise gains here...
        
        // 3. Set up a timer for the prediction step (e.g., at 100 Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&MyDroneNode::predict_step, this));

        // Create subscriptions for sensor data to trigger the correction step
        // odom_sub_ = this->create_subscription<...>(...);
        // imu_sub_ = this->create_subscription<...>(...);
    }

private:
    void predict_step()
    {
        // Get motor commands (u) and time delta (dt)
        Eigen::Matrix<double, 4, 1> u;
        u.setZero(); // Replace with actual motor commands
        double dt = 0.01; // Calculate actual dt

        // 4. Run prediction
        estimator_->predict(u, dt);

        // Run correction with the latest sensor data
        correct_step();
    }

    void correct_step()
    {
        // 5. Populate the measurement package with available data
        laser_uav_estimator::MeasurementPackage measurements;
        
        // Example: if you have a new odometry message
        // measurements.px4_odometry = latest_odom_msg_;

        // Example: if you have a new IMU message
        // measurements.imu = latest_imu_msg_;
        
        // 6. Run correction
        estimator_->correct(measurements);

        // 7. Get the latest state
        const auto& state = estimator_->get_state();
        RCLCPP_INFO(this->get_logger(), "Current Z position: %f", state(laser_uav_estimator::State::PZ));
    }

    std::unique_ptr<laser_uav_estimator::StateEstimator> estimator_;
    rclcpp::TimerBase::SharedPtr timer_;
    // Add member variables for subscriptions and to store latest messages
};
```

## Author & License

  * **Author**: Wagner Dantas Garcia
  * **Maintainer**: wagnergarcia@eng.ci.ufpb.br
  * **License**: Apache License 2.0
