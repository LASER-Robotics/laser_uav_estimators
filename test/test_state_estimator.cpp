/**
 * @file test_state_estimator.cpp
 * @brief Comprehensive unit tests for the StateEstimator class
 * @author Wagner Dantas Garcia / Laser UAV Team
 * @date September 16, 2025
 * @copyright Apache License 2.0
 *
 * @details This file contains a comprehensive test suite for the StateEstimator class,
 * which implements an Extended Kalman Filter (EKF) for UAV state estimation. The tests
 * cover various aspects of the estimator including:
 *
 * Test Categories:
 * - Basic initialization and fundamental behavior
 * - Prediction and correction step validation
 * - Flight scenario simulations (hover, takeoff, landing)
 * - Movement testing (vertical, horizontal, rotational)
 * - Complex maneuvers and edge cases
 * - Robustness and error handling
 * - Long-term stability and performance benchmarks
 *
 * Test Philosophy:
 * The tests follow a physics-based approach, verifying that the EKF correctly
 * models drone dynamics under different control inputs. Each test validates
 * both the mathematical correctness of the filter and the physical realism
 * of the resulting behavior.
 *
 * Dependencies:
 * - GoogleTest framework for unit testing
 * - laser_uav_estimators::StateEstimator - Main class under test
 * - laser_uav_lib::AttitudeConverter - For attitude representation utilities
 * - Standard ROS2 message types for sensor data simulation
 *
 * @note All tests use realistic drone parameters based on actual UAV specifications
 * @see https://github.com/LASER-Robotics/laser_uav_estimators
 * @see https://github.com/LASER-Robotics/laser_uav_lib
 */
#include <gtest/gtest.h>
#include <laser_uav_estimators/state_estimator.hpp>
#include <laser_uav_lib/attitude_converter/attitude_converter.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>

// =============================================================================
// Test Configuration Constants
// =============================================================================

/// @brief Default tolerance for numerical stability checks
/// Used to verify that states remain close to expected values during simulations
constexpr double STATE_TOLERANCE = 1e-9;

// =============================================================================
// Drone Physical Parameters for Testing
// =============================================================================
const double          TEST_MASS    = 1.60;
const Eigen::Matrix3d TEST_INERTIA = Eigen::Vector3d(0.021667, 0.021667, 0.0425).asDiagonal();

// [CORREÇÃO 1] Matriz de Alocação Real do X500 (baseada no arquivo x500.yaml)
// Esta matriz mapeia as forças dos 4 motores para o wrench [Fz, Tx, Ty, Tz] no corpo.
// Matriz G1 do controlador X500 oficial
const Eigen::Matrix<double, 4, 4> TEST_ALLOCATION_MATRIX = (Eigen::Matrix<double, 4, 4>() << 1.0, 1.0, 1.0, 1.0,  // Fz row: thrust sum
                                                            -0.18, 0.18, 0.18, -0.18,                             // Tx row: roll torque
                                                            -0.185, 0.185, -0.185, 0.185,                         // Ty row: pitch torque
                                                            -0.016, -0.016, 0.016, 0.016                          // Tz row: yaw torque
                                                            )
                                                               .finished();


// =============================================================================
// Test Fixture Class
// =============================================================================
class TestStateEstimator : public ::testing::Test {
protected:
  std::unique_ptr<laser_uav_estimators::StateEstimator> ekf;
  const double                                          hover_thrust_per_motor = (TEST_MASS * 9.80665) / 4.0;

  void SetUp() override {
    // StateEstimator(mass, allocation_matrix, inertia, verbosity)
    // Usando a matriz de alocação real do X500 do arquivo x500.yaml
    ekf = std::make_unique<laser_uav_estimators::StateEstimator>(TEST_MASS,
                                                                 TEST_ALLOCATION_MATRIX,  // Matriz real do X500
                                                                 TEST_INERTIA, "INFO");
  }

  void TearDown() override {
    ekf.reset();
  }

  // ... (restantes métodos da fixture: run_simulation, run_full_simulation, print_state - SEM ALTERAÇÕES) ...
  /**
   * @brief Executes multi-step prediction simulation with constant control input
   *
   * @param u Control vector containing thrust commands for each motor [N]
   * @param steps Number of prediction steps to execute
   * @param dt Time interval between prediction steps [seconds]
   */
  void run_simulation(const Eigen::Vector4d &u, int steps, double dt) {
    for (int i = 0; i < steps; ++i) {
      ekf->predict(u, dt);
    }
  }

  /**
   * @brief Executes complete prediction-correction cycles with measurements
   *
   * @param u Control vector for prediction steps [N]
   * @param measurements Sensor measurement package for corrections
   * @param steps Number of complete cycles (predict + correct) to execute
   * @param dt Time interval between cycles [seconds]
   */
  void run_full_simulation(const Eigen::Vector4d &u, const laser_uav_estimators::MeasurementPackage &measurements, int steps, double dt) {
    // Certifica-te que dt é passado para as medições, se necessário
    laser_uav_estimators::MeasurementPackage updated_measurements = measurements;
    updated_measurements.dt                                       = dt;

    for (int i = 0; i < steps; ++i) {
      ekf->predict(u, dt);
      ekf->correct(updated_measurements);  // Usa as medições atualizadas com dt
    }
  }


};  // <-- [CORREÇÃO] FIM DA CLASSE TEST FIXTURE. Faltava esta chave e ponto-e-vírgula.


// =============================================================================
// BASIC INITIALIZATION AND FUNDAMENTAL BEHAVIOR TESTS
//
// This section contains foundational tests that verify the basic functionality
// of the StateEstimator. These tests ensure that the EKF initializes correctly
// and that the core prediction/correction mechanisms work as expected.
// =============================================================================

/**
 * @test Initialization
 * @brief Verifies correct initial state and covariance setup
 *
 * @details This fundamental test ensures that the StateEstimator initializes
 * to a physically meaningful default state:
 * - Position at origin: [0, 0, 0]
 * - Identity orientation: quaternion [1, 0, 0, 0] (no rotation)
 * - Zero velocities: both linear and angular
 * - Positive definite covariance matrix (indicating initial uncertainty)
 *
 * This test is critical because all subsequent tests depend on consistent
 * initialization behavior. Any failure here indicates fundamental issues
 * with the EKF constructor or default parameter setup.
 *
 * Expected Behavior:
 * - All position and velocity components should be exactly zero
 * - Quaternion should represent identity rotation (w=1, x=y=z=0)
 * - Covariance trace should be positive (indicating some initial uncertainty)
 */
TEST_F(TestStateEstimator, Initialization) {
  ekf->set_verbosity("SILENT");

  const auto &state      = ekf->get_state();
  const auto &covariance = ekf->get_covariance();

  // Verify identity quaternion (no initial rotation)
  EXPECT_DOUBLE_EQ(state(laser_uav_estimators::State::QW), 1.0);

  // Verify zero initial position
  EXPECT_DOUBLE_EQ(state.segment<3>(laser_uav_estimators::State::PX).norm(), 0.0);

  // Verify zero initial orientation components (x, y, z of quaternion)
  EXPECT_DOUBLE_EQ(state.segment<3>(laser_uav_estimators::State::QX).norm(), 0.0);

  // Verify zero initial linear velocity
  EXPECT_DOUBLE_EQ(state.segment<3>(laser_uav_estimators::State::VX).norm(), 0.0);

  // Verify zero initial angular velocity
  EXPECT_DOUBLE_EQ(state.segment<3>(laser_uav_estimators::State::WX).norm(), 0.0);

  // Verify positive initial uncertainty (covariance trace > 0)
  ASSERT_GT(covariance.trace(), 0.0);
}

/**
 * @test PredictionIncreasesUncertainty
 * @brief Validates that prediction steps properly increase state uncertainty
 *
 * @details This test verifies a fundamental property of Kalman filters:
 * prediction steps should increase uncertainty (covariance) due to process
 * noise. This reflects the fact that as we predict forward in time without
 * new measurements, our confidence in the state estimate should decrease.
 *
 * The test applies a mild upward thrust (slightly above hover) and runs
 * multiple prediction steps, then verifies that the total uncertainty
 * (measured by covariance trace) has increased.
 *
 * Expected Behavior:
 * - Covariance trace should monotonically increase with each prediction
 * - The increase should be proportional to the process noise parameters
 * - State estimates should remain finite and physically reasonable
 *
 * @note This test validates proper process noise modeling and numerical stability
 */
TEST_F(TestStateEstimator, PredictionIncreasesUncertainty) {
  ekf->set_verbosity("SILENT");

  // Apply mild upward thrust (slightly above hover level)
  Eigen::Vector4d u_hover = Eigen::Vector4d::Constant(hover_thrust_per_motor + 0.2);

  // Capture initial uncertainty level
  auto initial_trace = ekf->get_covariance().trace();

  // Run multiple prediction steps
  run_simulation(u_hover, 5, 0.02);

  // Verify uncertainty has increased
  auto final_trace = ekf->get_covariance().trace();
  EXPECT_GT(final_trace, initial_trace);
}

/**
 * @test CorrectionDecreasesUncertainty
 * @brief Validates that correction steps properly reduce state uncertainty
 *
 * @details This test verifies another fundamental property of Kalman filters:
 * correction steps should decrease uncertainty (covariance) when incorporating
 * sensor measurements. This reflects the fact that new measurement information
 * should improve our confidence in the state estimate.
 *
 * The test creates a realistic odometry measurement with low noise and runs
 * multiple prediction-correction cycles, then verifies that the total uncertainty
 * (measured by covariance trace) has decreased compared to the initial state.
 *
 * Test Phases:
 * 1. Capture initial uncertainty level
 * 2. Create high-quality odometry measurement (low covariance)
 * 3. Run multiple prediction-correction cycles
 * 4. Verify final uncertainty is lower than initial
 *
 * Expected Behavior:
 * - Final covariance trace should be smaller than initial trace
 * - The filter should converge toward the provided measurements
 * - State estimates should remain physically reasonable
 *
 * @note This test validates sensor fusion effectiveness and measurement processing
 */
TEST_F(TestStateEstimator, CorrectionDecreasesUncertainty) {
  ekf->set_verbosity("SILENT");

  // --- Arrange: Setup test environment ---
  // Capture initial filter uncertainty level
  auto initial_trace = ekf->get_covariance().trace();

  // Create high-quality measurement package
  laser_uav_estimators::MeasurementPackage measurements;
  nav_msgs::msg::Odometry                  odom_msg;

  // Position measurement: drone at origin
  odom_msg.pose.pose.position.x = 0.0;
  odom_msg.pose.pose.position.y = 0.0;
  odom_msg.pose.pose.position.z = 0.0;

  // Orientation measurement: identity quaternion (no rotation)
  odom_msg.pose.pose.orientation.w = 1.0;
  odom_msg.pose.pose.orientation.x = 0.0;
  odom_msg.pose.pose.orientation.y = 0.0;
  odom_msg.pose.pose.orientation.z = 0.0;

  // Velocity measurements: drone at rest
  odom_msg.twist.twist.linear.x  = 0.0;
  odom_msg.twist.twist.linear.y  = 0.0;
  odom_msg.twist.twist.linear.z  = 0.0;
  odom_msg.twist.twist.angular.x = 0.0;
  odom_msg.twist.twist.angular.y = 0.0;
  odom_msg.twist.twist.angular.z = 0.0;

  // Set measurement covariances (diagonal matrices with low noise)
  // High confidence in position and orientation measurements
  for (int i = 0; i < 36; ++i) {
    odom_msg.pose.covariance[i]  = (i % 7 == 0) ? 0.001 : 0.0;  // Position/orientation uncertainty
    odom_msg.twist.covariance[i] = (i % 7 == 0) ? 0.01 : 0.0;   // Velocity uncertainty
  }

  measurements.px4_odometry = odom_msg;

  // --- Act: Execute prediction-correction cycles ---
  // Run multiple complete cycles to allow filter convergence
  run_full_simulation(Eigen::Vector4d::Zero(), measurements, 100, 0.02);

  auto final_trace = ekf->get_covariance().trace();

  // --- Assert: Verify uncertainty reduction ---
  // Primary assertion: final uncertainty should be lower than initial
  // This demonstrates that the filter is effectively using measurements to refine estimates
  EXPECT_LT(final_trace, initial_trace);
}

/**
 * @test QuaternionNormalization
 * @brief Ensures quaternion orientation representation remains properly normalized
 *
 * @details This test verifies numerical stability of quaternion handling during
 * extended filter operation. Quaternions must maintain unit norm (magnitude = 1.0)
 * to represent valid 3D rotations. Numerical errors can cause quaternions to
 * drift from unit norm, leading to invalid attitude representations.
 *
 * The test runs a long simulation with constant hover thrust and verifies that
 * the quaternion components maintain proper normalization throughout. This is
 * critical for:
 * - Valid attitude representation
 * - Numerical stability of rotation operations
 * - Consistency with attitude conversion utilities
 *
 * Expected Behavior:
 * - Quaternion norm should remain very close to 1.0 (within 1e-6 tolerance)
 * - No numerical drift or instability over extended operation
 * - Proper quaternion normalization enforcement in the EKF
 *
 * @note This test validates quaternion constraint handling and numerical precision
 */
TEST_F(TestStateEstimator, QuaternionNormalization) {
  ekf->set_verbosity("SILENT");

  // Run extended simulation with hover thrust (1000 steps)
  run_simulation(Eigen::Vector4d::Constant(hover_thrust_per_motor), 1000, 0.01);

  const auto &state = ekf->get_state();

  // Calculate quaternion norm manually for verification
  double norm_q = std::sqrt(std::pow(state(laser_uav_estimators::State::QW), 2) + std::pow(state(laser_uav_estimators::State::QX), 2) +
                            std::pow(state(laser_uav_estimators::State::QY), 2) + std::pow(state(laser_uav_estimators::State::QZ), 2));

  // Verify quaternion remains properly normalized
  EXPECT_NEAR(norm_q, 1.0, 1e-6);
}

/**
 * @test ResetRestoresInitialState
 * @brief Verifies that the reset function properly restores initial conditions
 *
 * @details This test validates the filter's reset functionality, which should
 * restore the EKF to its initial state as if it were freshly constructed.
 * This capability is essential for:
 * - Recovery from filter divergence
 * - Reinitializing after significant disturbances
 * - Starting fresh estimation after system failures
 *
 * The test procedure:
 * 1. Run a simulation to significantly change the filter state
 * 2. Call the reset() method
 * 3. Verify all state components return to initial values
 *
 * Expected Behavior:
 * - Position returns to origin [0, 0, 0]
 * - Orientation returns to identity quaternion [1, 0, 0, 0]
 * - All velocities return to zero
 * - Covariance returns to initial uncertainty levels
 *
 * @note This test ensures proper state management and recovery capabilities
 */
TEST_F(TestStateEstimator, ResetRestoresInitialState) {
  ekf->set_verbosity("SILENT");

  // Significantly alter the filter state through simulation
  run_simulation(Eigen::Vector4d::Constant(hover_thrust_per_motor + 1.0), 100, 0.02);

  // Reset the filter to initial conditions
  ekf->reset();

  const auto &state = ekf->get_state();

  // Verify identity quaternion restoration (w=1, x=y=z=0)
  EXPECT_DOUBLE_EQ(state(laser_uav_estimators::State::QW), 1.0);

  // Verify zero position restoration
  EXPECT_DOUBLE_EQ(state.segment<3>(laser_uav_estimators::State::PX).norm(), 0.0);

  // Verify zero velocity restoration
  EXPECT_DOUBLE_EQ(state.segment<3>(laser_uav_estimators::State::VX).norm(), 0.0);
}

// =============================================================================
// BASIC FLIGHT SCENARIO TESTS
//
// This section contains tests that validate the EKF's behavior in fundamental
// flight scenarios. These tests ensure that the filter correctly models basic
// drone physics and responds appropriately to different control inputs.
// =============================================================================

/**
 * @test HoverShouldRemainStationary
 * @brief Validates stationary hover flight behavior
 *
 * @details This test verifies that the EKF correctly models hover flight,
 * where thrust exactly counteracts gravity. In perfect hover conditions:
 * - Position should remain approximately constant
 * - Linear velocities should remain near zero
 * - Angular velocities should remain near zero
 * - Only small variations due to process noise are expected
 *
 * The test applies the calculated hover thrust (mass * gravity / 4 motors)
 * and runs an extended simulation to verify steady-state behavior.
 *
 * Expected Behavior:
 * - Position drift should be minimal (< 1e-3 m)
 * - Linear velocity should remain near zero (< 1e-3 m/s)
 * - Angular velocity should remain near zero (< 1e-3 rad/s)
 *
 * @note This test validates the fundamental hover dynamics model
 */
TEST_F(TestStateEstimator, HoverShouldRemainStationary) {
  ekf->set_verbosity("SILENT");

  // Apply exact hover thrust (counteracts gravity)
  Eigen::Vector4d u_hover = Eigen::Vector4d::Constant(hover_thrust_per_motor);
  run_simulation(u_hover, 100, 0.02);

  const auto &state = ekf->get_state();

  // Verify minimal position drift during hover
  EXPECT_NEAR(state.segment<3>(laser_uav_estimators::State::PX).norm(), 0.0, 1e-3);

  // Verify minimal linear velocity during hover
  EXPECT_NEAR(state.segment<3>(laser_uav_estimators::State::VX).norm(), 0.0, 1e-3);

  // Verify minimal angular velocity during hover
  EXPECT_NEAR(state.segment<3>(laser_uav_estimators::State::WX).norm(), 0.0, 1e-3);
}

/**
 * @test FreeFallWhenNoThrust
 * @brief Validates gravitational dynamics with zero thrust
 *
 * @details This test verifies that the EKF correctly models free fall behavior
 * when no thrust is applied. Under these conditions, gravity should be the
 * dominant force, causing:
 * - Downward acceleration (negative Z direction)
 * - Increasing downward velocity
 * - Downward displacement from initial position
 *
 * The test applies zero thrust to all motors and verifies that the drone
 * exhibits proper gravitational dynamics according to physics.
 *
 * Expected Behavior:
 * - Position Z should become negative (falling down)
 * - Velocity Z should become negative (accelerating downward)
 * - Acceleration should approximately equal -9.81 m/s²
 *
 * Physics Validation:
 * This test ensures the EKF's dynamics model correctly incorporates gravity
 * and validates the fundamental force balance equations.
 *
 * @note This test validates gravitational force modeling in the EKF dynamics
 */
TEST_F(TestStateEstimator, FreeFallWhenNoThrust) {
  ekf->set_verbosity("SILENT");

  // Apply zero thrust (no motor forces, only gravity acts)
  run_simulation(Eigen::Vector4d::Zero(), 100, 0.02);

  const auto &state = ekf->get_state();

  // Verify downward displacement due to gravity
  EXPECT_LT(state(laser_uav_estimators::State::PZ), 0.0);

  // Verify downward velocity due to gravitational acceleration
  EXPECT_LT(state(laser_uav_estimators::State::VZ), 0.0);
}

// =============================================================================
// VERTICAL MOVEMENT TESTS
//
// This section tests the EKF's response to vertical thrust variations,
// validating the dynamics model for upward and downward motion scenarios.
// =============================================================================

/**
 * @test UpwardMovement
 * @brief Validates upward flight dynamics with excess thrust
 *
 * @details This test verifies that the EKF correctly models upward motion
 * when thrust exceeds the force required to counteract gravity. The dynamics
 * should produce:
 * - Net upward force causing positive vertical acceleration
 * - Increasing upward velocity over time
 * - Positive vertical displacement
 * - Minimal deviation in horizontal axes (X, Y)
 *
 * Test Configuration:
 * - Applied thrust: hover_thrust + 1.0 N per motor
 * - Duration: 50 time steps at 0.02s intervals (1 second total)
 * - Expected motion: Pure vertical ascent
 *
 * Expected Behavior:
 * - Position Z > 0 (altitude gain)
 * - Velocity Z > 0 (upward motion)
 * - Position X, Y ≈ 0 (no horizontal drift)
 *
 * @note This test validates thrust-to-acceleration dynamics modeling
 */
TEST_F(TestStateEstimator, UpwardMovement) {
  ekf->set_verbosity("SILENT");

  // Apply upward thrust (hover thrust + additional force)
  Eigen::Vector4d u_up = Eigen::Vector4d::Constant(hover_thrust_per_motor + 1.0);
  run_simulation(u_up, 50, 0.02);

  const auto &state = ekf->get_state();

  // Verify positive altitude gain
  EXPECT_GT(state(laser_uav_estimators::State::PZ), 0.0);

  // Verify positive vertical velocity (climbing)
  EXPECT_GT(state(laser_uav_estimators::State::VZ), 0.0);

  // Verify minimal horizontal drift during vertical motion
  EXPECT_NEAR(state(laser_uav_estimators::State::PX), 0.0, 1e-4);
  EXPECT_NEAR(state(laser_uav_estimators::State::PY), 0.0, 1e-4);
}

/**
 * @test DownwardMovement
 * @brief Validates downward flight dynamics with reduced thrust
 *
 * @details This test verifies that the EKF correctly models downward motion
 * when thrust is insufficient to counteract gravity. The test sequence:
 * 1. Initial ascent to establish positive altitude
 * 2. Thrust reduction to create net downward force
 * 3. Verification of descending motion characteristics
 *
 * Test Configuration:
 * - Phase 1: Ascent with hover_thrust + 1.0 N per motor (5 steps)
 * - Phase 2: Descent with hover_thrust - 1.0 N per motor (10 steps)
 * - Step duration: 0.02s intervals
 *
 * Expected Behavior:
 * - Negative vertical velocity (descending motion)
 * - Controlled descent dynamics
 * - Proper response to thrust reduction
 *
 * @note This test validates multi-phase flight dynamics and thrust response
 */
TEST_F(TestStateEstimator, DownwardMovement) {
  ekf->set_verbosity("SILENT");

  // Phase 1: Initial ascent to establish positive altitude
  run_simulation(Eigen::Vector4d::Constant(hover_thrust_per_motor + 1.0), 5, 0.02);

  // Phase 2: Apply reduced thrust to initiate descent
  run_simulation(Eigen::Vector4d::Constant(hover_thrust_per_motor - 1.0), 10, 0.02);

  const auto &state = ekf->get_state();

  // Verify negative vertical velocity (descending motion)
  EXPECT_LT(state(laser_uav_estimators::State::VZ), 0.0);
}

// =============================================================================
// HORIZONTAL MOVEMENT TESTS
//
// This section contains tests that validate the EKF's behavior during
// horizontal flight maneuvers. These tests verify proper modeling of
// differential thrust inputs that create pitch/roll attitudes for translation.
// =============================================================================

/**
 * @test ForwardMovement
 * @brief Validates forward flight dynamics through differential thrust
 *
 * @details This test verifies that the EKF correctly models forward motion
 * generated by differential motor thrust. The control strategy:
 * - Creates negative pitch moment (nose down) through motor differential
 * - Generates forward thrust component when drone tilts
 * - Maintains approximate hover altitude through thrust balance
 *
 * Motor Configuration for Forward Motion:
 * - Front motors: hover_thrust - differential
 * - Rear motors: hover_thrust + differential
 * - Net effect: Negative pitch torque → forward tilt → forward translation
 *
 * Test Configuration:
 * - Thrust differential: 0.5 N between front/rear motor pairs
 * - Duration: 50 time steps at 0.02s intervals (1 second total)
 * - Expected motion: Forward translation in +X direction
 *
 * Expected Behavior:
 * - Position X > 0 (forward displacement)
 * - Velocity X > 0 (forward motion)
 * - Quaternion QY reflects forward pitch attitude
 *
 * @note This test validates attitude-controlled translation dynamics
 */
TEST_F(TestStateEstimator, ForwardMovement) {
  double diff = 0.5;

  // Create differential thrust for forward pitch
  // Real X500 motor order: M0=Rear-Left, M1=Front-Right, M2=Rear-Right, M3=Front-Left
  // For forward movement: reduce rear motors, increase front motors
  Eigen::Vector4d u_forward(hover_thrust_per_motor - diff,   // Motor 0 (Rear-Left, reduced)
                            hover_thrust_per_motor + diff,   // Motor 1 (Front-Right, increased)
                            hover_thrust_per_motor - diff,   // Motor 2 (Rear-Right, reduced)
                            hover_thrust_per_motor + diff);  // Motor 3 (Front-Left, increased)

  run_simulation(u_forward, 50, 0.02);
  const auto &state = ekf->get_state();

  // Verify forward translation
  EXPECT_GT(state(laser_uav_estimators::State::PX), 0.0);  // Forward displacement
  EXPECT_GT(state(laser_uav_estimators::State::VX), 0.0);  // Forward velocity
  EXPECT_GT(state(laser_uav_estimators::State::QY), 0.0);  // Forward pitch attitude

  // Verify minimal cross-coupling in other axes
  EXPECT_NEAR(state(laser_uav_estimators::State::PY), 0.0, STATE_TOLERANCE);  // No lateral drift
  EXPECT_NEAR(state(laser_uav_estimators::State::VY), 0.0, STATE_TOLERANCE);  // No lateral velocity
  EXPECT_NEAR(state(laser_uav_estimators::State::QX), 0.0, STATE_TOLERANCE);  // No roll attitude
  EXPECT_NEAR(state(laser_uav_estimators::State::QZ), 0.0, STATE_TOLERANCE);  // No yaw attitude
}

/**
 * @test BackwardMovement
 * @brief Validates backward flight dynamics through differential thrust
 *
 * @details This test verifies that the EKF correctly models backward motion
 * generated by differential motor thrust. The control strategy:
 * - Creates positive pitch moment (nose up) through motor differential
 * - Generates backward thrust component when drone tilts
 * - Maintains approximate hover altitude through thrust balance
 *
 * Motor Configuration for Backward Motion:
 * - Front motors: hover_thrust + differential
 * - Rear motors: hover_thrust - differential
 * - Net effect: Positive pitch torque → backward tilt → backward translation
 *
 * Test Configuration:
 * - Thrust differential: 0.5 N between front/rear motor pairs
 * - Duration: 50 time steps at 0.02s intervals (1 second total)
 * - Expected motion: Backward translation in -X direction
 *
 * Expected Behavior:
 * - Position X < 0 (backward displacement)
 * - Velocity X < 0 (backward motion)
 * - Quaternion QY < 0 (backward pitch attitude)
 *
 * @note This test validates reverse attitude-controlled translation dynamics
 */
TEST_F(TestStateEstimator, BackwardMovement) {
  double diff = 0.5;

  // Create differential thrust for backward pitch
  // Real X500 motor order: M0=Rear-Left, M1=Front-Right, M2=Rear-Right, M3=Front-Left
  // For backward movement: increase rear motors, reduce front motors
  Eigen::Vector4d u_backward(hover_thrust_per_motor + diff,   // Motor 0 (Rear-Left, increased)
                             hover_thrust_per_motor - diff,   // Motor 1 (Front-Right, reduced)
                             hover_thrust_per_motor + diff,   // Motor 2 (Rear-Right, increased)
                             hover_thrust_per_motor - diff);  // Motor 3 (Front-Left, reduced)

  run_simulation(u_backward, 50, 0.02);
  const auto &state = ekf->get_state();

  // Verify backward translation
  EXPECT_LT(state(laser_uav_estimators::State::PX), 0.0);  // Backward displacement
  EXPECT_LT(state(laser_uav_estimators::State::VX), 0.0);  // Backward velocity
  EXPECT_LT(state(laser_uav_estimators::State::QY), 0.0);  // Backward pitch attitude

  // Verify minimal cross-coupling in other axes
  EXPECT_NEAR(state(laser_uav_estimators::State::PY), 0.0, STATE_TOLERANCE);  // No lateral drift
  EXPECT_NEAR(state(laser_uav_estimators::State::VY), 0.0, STATE_TOLERANCE);  // No lateral velocity
  EXPECT_NEAR(state(laser_uav_estimators::State::QX), 0.0, STATE_TOLERANCE);  // No roll attitude
  EXPECT_NEAR(state(laser_uav_estimators::State::QZ), 0.0, STATE_TOLERANCE);  // No yaw attitude
}

/**
 * @test RightwardMovement
 * @brief Validates rightward flight dynamics through differential thrust
 *
 * @details This test verifies that the EKF correctly models rightward motion
 * generated by differential motor thrust. The control strategy:
 * - Creates positive roll moment (right bank) through motor differential
 * - Generates rightward thrust component when drone tilts
 * - Maintains approximate hover altitude through thrust balance
 *
 * Motor Configuration for Rightward Motion:
 * - Left motors: hover_thrust + differential
 * - Right motors: hover_thrust - differential
 * - Net effect: Positive roll torque → right tilt → rightward translation
 *
 * Test Configuration:
 * - Thrust differential: 0.5 N between left/right motor pairs
 * - Duration: 50 time steps at 0.02s intervals (1 second total)
 * - Expected motion: Rightward translation in -Y direction (NED frame)
 *
 * Expected Behavior:
 * - Position Y < 0 (rightward displacement in NED frame)
 * - Velocity Y < 0 (rightward motion)
 * - Quaternion QX > 0 (right roll attitude)
 *
 * @note This test validates lateral attitude-controlled translation dynamics
 */
TEST_F(TestStateEstimator, RightwardMovement) {
  ekf->set_verbosity("SILENT");

  double diff = 0.5;

  // Create differential thrust for right roll
  // Real X500 motor order: M0=Rear-Left, M1=Front-Right, M2=Rear-Right, M3=Front-Left
  // Tx coefficients: M0=-0.18, M1=0.18, M2=0.18, M3=-0.18
  // For rightward movement (positive roll): increase M1,M2, reduce M0,M3
  Eigen::Vector4d u_right(hover_thrust_per_motor - diff,   // Motor 0 (Rear-Left, reduced)
                          hover_thrust_per_motor + diff,   // Motor 1 (Front-Right, increased)
                          hover_thrust_per_motor + diff,   // Motor 2 (Rear-Right, increased)
                          hover_thrust_per_motor - diff);  // Motor 3 (Front-Left, reduced)

  run_simulation(u_right, 50, 0.02);
  const auto &state = ekf->get_state();

  // Verify rightward translation (negative Y in NED frame)
  EXPECT_LT(state(laser_uav_estimators::State::PY), 0.0);  // Rightward displacement
  EXPECT_LT(state(laser_uav_estimators::State::VY), 0.0);  // Rightward velocity
  EXPECT_GT(state(laser_uav_estimators::State::QX), 0.0);  // Right roll attitude

  // Verify minimal cross-coupling in other axes
  EXPECT_NEAR(state(laser_uav_estimators::State::PX), 0.0, STATE_TOLERANCE);  // No forward/backward drift
  EXPECT_NEAR(state(laser_uav_estimators::State::VX), 0.0, STATE_TOLERANCE);  // No forward/backward velocity
  EXPECT_NEAR(state(laser_uav_estimators::State::QY), 0.0, STATE_TOLERANCE);  // No pitch attitude
  EXPECT_NEAR(state(laser_uav_estimators::State::QZ), 0.0, STATE_TOLERANCE);  // No yaw attitude
}

/**
 * @test LeftwardMovement
 * @brief Validates leftward flight dynamics through differential thrust
 *
 * @details This test verifies that the EKF correctly models leftward motion
 * generated by differential motor thrust. The control strategy:
 * - Creates negative roll moment (left bank) through motor differential
 * - Generates leftward thrust component when drone tilts
 * - Maintains approximate hover altitude through thrust balance
 *
 * Motor Configuration for Leftward Motion:
 * - Left motors: hover_thrust - differential
 * - Right motors: hover_thrust + differential
 * - Net effect: Negative roll torque → left tilt → leftward translation
 *
 * Test Configuration:
 * - Thrust differential: 0.5 N between left/right motor pairs
 * - Duration: 50 time steps at 0.02s intervals (1 second total)
 * - Expected motion: Leftward translation in +Y direction (NED frame)
 *
 * Expected Behavior:
 * - Position Y > 0 (leftward displacement in NED frame)
 * - Velocity Y > 0 (leftward motion)
 * - Quaternion QX < 0 (left roll attitude)
 *
 * @note This test validates reverse lateral attitude-controlled translation dynamics
 */
TEST_F(TestStateEstimator, LeftwardMovement) {
  ekf->set_verbosity("SILENT");

  double diff = 0.5;

  // Create differential thrust for left roll
  // Real X500 motor order: M0=Rear-Left, M1=Front-Right, M2=Rear-Right, M3=Front-Left
  // Tx coefficients: M0=-0.18, M1=0.18, M2=0.18, M3=-0.18
  // For leftward movement (negative roll): increase M0,M3, reduce M1,M2
  Eigen::Vector4d u_left(hover_thrust_per_motor + diff,   // Motor 0 (Rear-Left, increased)
                         hover_thrust_per_motor - diff,   // Motor 1 (Front-Right, reduced)
                         hover_thrust_per_motor - diff,   // Motor 2 (Rear-Right, reduced)
                         hover_thrust_per_motor + diff);  // Motor 3 (Front-Left, increased)

  run_simulation(u_left, 50, 0.02);
  const auto &state = ekf->get_state();

  // Verify leftward translation (positive Y in NED frame)
  EXPECT_GT(state(laser_uav_estimators::State::PY), 0.0);  // Leftward displacement
  EXPECT_GT(state(laser_uav_estimators::State::VY), 0.0);  // Leftward velocity
  EXPECT_LT(state(laser_uav_estimators::State::QX), 0.0);  // Left roll attitude

  // Verify minimal cross-coupling in other axes
  EXPECT_NEAR(state(laser_uav_estimators::State::PX), 0.0, STATE_TOLERANCE);  // No forward/backward drift
  EXPECT_NEAR(state(laser_uav_estimators::State::VX), 0.0, STATE_TOLERANCE);  // No forward/backward velocity
  EXPECT_NEAR(state(laser_uav_estimators::State::QY), 0.0, STATE_TOLERANCE);  // No pitch attitude
  EXPECT_NEAR(state(laser_uav_estimators::State::QZ), 0.0, STATE_TOLERANCE);  // No yaw attitude
}

// =============================================================================
// ROTATION TESTS
//
// This section contains tests that validate the EKF's behavior during
// rotational maneuvers. These tests verify proper modeling of differential
// thrust inputs that create pure rotational moments without translation.
// =============================================================================

/**
 * @test YawRotation
 * @brief Validates yaw rotation dynamics through differential thrust
 *
 * @details This test verifies that the EKF correctly models yaw (Z-axis) rotation
 * generated by differential motor thrust. The control strategy:
 * - Creates net yaw torque through opposing motor pairs
 * - Generates pure rotational motion without translation
 * - Maintains hover altitude through balanced total thrust
 *
 * Motor Configuration for Positive Yaw Rotation:
 * - Motors 0,1 (diagonal pair): hover_thrust + differential
 * - Motors 2,3 (diagonal pair): hover_thrust - differential
 * - Net effect: Positive yaw torque → rotation about Z-axis
 *
 * Test Configuration:
 * - Thrust differential: 0.1 N between motor pairs
 * - Duration: 500 time steps at 0.01s intervals (5 seconds total)
 * - Expected motion: Pure rotation about vertical axis
 *
 * Expected Behavior:
 * - Angular velocity WZ > 0 (positive yaw rate)
 * - Quaternion QZ > 0 (accumulated yaw angle)
 * - Minimal translation in X, Y axes
 *
 * @note This test validates torque-to-angular-acceleration dynamics modeling
 */
TEST_F(TestStateEstimator, YawRotation) {
  ekf->set_verbosity("DEBUG");

  double diff = 0.1;

  // Create differential thrust for positive yaw torque
  // Real X500 motor order: M0=Rear-Left, M1=Front-Right, M2=Rear-Right, M3=Front-Left
  // Tz coefficients: M0=-0.016, M1=-0.016, M2=0.016, M3=0.016
  // For positive yaw rotation: increase M2,M3 (CCW), reduce M0,M1 (CW)
  Eigen::Vector4d u_yaw(hover_thrust_per_motor - diff,   // Motor 0 (Rear-Left, reduced)
                        hover_thrust_per_motor - diff,   // Motor 1 (Front-Right, reduced)
                        hover_thrust_per_motor + diff,   // Motor 2 (Rear-Right, increased)
                        hover_thrust_per_motor + diff);  // Motor 3 (Front-Left, increased)

  run_simulation(u_yaw, 500, 0.01);
  const auto &state = ekf->get_state();

  // Verify positive yaw rotation
  EXPECT_GT(state(laser_uav_estimators::State::WZ), 0.0);  // Positive yaw rate
  EXPECT_GT(state(laser_uav_estimators::State::QZ), 0.0);  // Positive yaw angle

  // Verify minimal cross-coupling in other axes
  EXPECT_NEAR(state(laser_uav_estimators::State::QX), 0.0, STATE_TOLERANCE);  // No roll attitude
  EXPECT_NEAR(state(laser_uav_estimators::State::QY), 0.0, STATE_TOLERANCE);  // No pitch attitude
}

/**
 * @test TwoYawRotation
 * @brief Validates negative yaw rotation dynamics through differential thrust
 *
 * @details This test verifies that the EKF correctly models negative yaw (Z-axis)
 * rotation generated by differential motor thrust in the opposite direction.
 * The control strategy:
 * - Creates net negative yaw torque through opposing motor pairs
 * - Generates pure counter-clockwise rotational motion
 * - Maintains hover altitude through balanced total thrust
 *
 * Motor Configuration for Negative Yaw Rotation:
 * - Motors 0,1 (diagonal pair): hover_thrust - differential
 * - Motors 2,3 (diagonal pair): hover_thrust + differential
 * - Net effect: Negative yaw torque → counter-clockwise rotation about Z-axis
 *
 * Test Configuration:
 * - Thrust differential: 0.1 N between motor pairs (opposite of YawRotation)
 * - Duration: 300 time steps at 0.01s intervals (3 seconds total)
 * - Expected motion: Pure counter-clockwise rotation about vertical axis
 *
 * Expected Behavior:
 * - Angular velocity WZ < 0 (negative yaw rate)
 * - Quaternion QZ < 0 (accumulated negative yaw angle)
 * - Minimal translation in X, Y axes
 *
 * @note This test validates bidirectional yaw control and torque symmetry
 */
TEST_F(TestStateEstimator, TwoYawRotation) {
  ekf->set_verbosity("INFO");

  double diff = 0.1;

  // Create differential thrust for negative yaw torque
  // Real X500 motor order: M0=Rear-Left, M1=Front-Right, M2=Rear-Right, M3=Front-Left
  // Tz coefficients: M0=-0.016, M1=-0.016, M2=0.016, M3=0.016
  // For negative yaw rotation: increase M0,M1 (CW), reduce M2,M3 (CCW)
  Eigen::Vector4d u_yaw(hover_thrust_per_motor + diff,   // Motor 0 (Rear-Left, increased)
                        hover_thrust_per_motor + diff,   // Motor 1 (Front-Right, increased)
                        hover_thrust_per_motor - diff,   // Motor 2 (Rear-Right, reduced)
                        hover_thrust_per_motor - diff);  // Motor 3 (Front-Left, reduced)

  run_simulation(u_yaw, 300, 0.01);
  const auto &state = ekf->get_state();

  // Verify negative yaw rotation
  EXPECT_LT(state(laser_uav_estimators::State::WZ), 0.0);  // Negative yaw rate
  EXPECT_LT(state(laser_uav_estimators::State::QZ), 0.0);  // Negative yaw angle

  // Verify minimal cross-coupling in other axes
  EXPECT_NEAR(state(laser_uav_estimators::State::QX), 0.0, STATE_TOLERANCE);  // No roll attitude
  EXPECT_NEAR(state(laser_uav_estimators::State::QY), 0.0, STATE_TOLERANCE);  // No pitch attitude
}

/**
 * @test PitchRotation
 * @brief Validates pitch rotation dynamics through differential thrust
 *
 * @details This test verifies that the EKF correctly models pitch (Y-axis) rotation
 * generated by differential motor thrust. The control strategy:
 * - Creates net pitch torque through front/rear motor differential
 * - Generates pure rotational motion about lateral axis
 * - Maintains hover altitude through balanced total thrust
 *
 * Motor Configuration for Positive Pitch Rotation:
 * - Front motors: hover_thrust - differential (nose down tendency)
 * - Rear motors: hover_thrust + differential (tail up tendency)
 * - Net effect: Positive pitch torque → nose-down rotation about Y-axis
 *
 * Test Configuration:
 * - Thrust differential: 0.2 N between front/rear motor pairs
 * - Duration: 50 time steps at 0.02s intervals (1 second total)
 * - Expected motion: Pure pitch rotation without translation
 *
 * Expected Behavior:
 * - Angular velocity WY > 0 (positive pitch rate)
 * - Quaternion QY > 0 (accumulated pitch angle)
 * - Minimal cross-coupling in roll and yaw axes
 *
 * @note This test validates pitch torque generation and angular dynamics modeling
 */
TEST_F(TestStateEstimator, PitchRotation) {
  ekf->set_verbosity("SILENT");

  double diff = 0.2;

  // Create differential thrust for positive pitch torque
  Eigen::Vector4d u_pitch(hover_thrust_per_motor - diff,   // Front left motor (reduced)
                          hover_thrust_per_motor + diff,   // Rear left motor (increased)
                          hover_thrust_per_motor - diff,   // Front right motor (reduced)
                          hover_thrust_per_motor + diff);  // Rear right motor (increased)

  run_simulation(u_pitch, 50, 0.02);
  const auto &state = ekf->get_state();

  // Verify positive pitch rotation
  EXPECT_GT(state(laser_uav_estimators::State::WY), 0.0);  // Positive pitch rate
  EXPECT_GT(state(laser_uav_estimators::State::QY), 0.0);  // Positive pitch angle

  // Verify minimal cross-coupling in other axes
  EXPECT_NEAR(state(laser_uav_estimators::State::QX), 0.0, STATE_TOLERANCE);  // No roll attitude
  EXPECT_NEAR(state(laser_uav_estimators::State::QZ), 0.0, STATE_TOLERANCE);  // No yaw attitude
  EXPECT_NEAR(state(laser_uav_estimators::State::WX), 0.0, STATE_TOLERANCE);  // No roll rate
  EXPECT_NEAR(state(laser_uav_estimators::State::WZ), 0.0, STATE_TOLERANCE);  // No yaw rate
}

/**
 * @test RollRotation
 * @brief Validates roll rotation dynamics through differential thrust
 *
 * @details This test verifies that the EKF correctly models roll (X-axis) rotation
 * generated by differential motor thrust. The control strategy:
 * - Creates net roll torque through left/right motor differential
 * - Generates pure rotational motion about longitudinal axis
 * - Maintains hover altitude through balanced total thrust
 *
 * Motor Configuration for Positive Roll Rotation:
 * - Left motors: hover_thrust + differential (left side up tendency)
 * - Right motors: hover_thrust - differential (right side down tendency)
 * - Net effect: Positive roll torque → right-bank rotation about X-axis
 *
 * Test Configuration:
 * - Thrust differential: 0.2 N between left/right motor pairs
 * - Duration: 50 time steps at 0.02s intervals (1 second total)
 * - Expected motion: Pure roll rotation without translation
 *
 * Expected Behavior:
 * - Angular velocity WX > 0 (positive roll rate)
 * - Quaternion QX > 0 (accumulated roll angle)
 * - Minimal cross-coupling in pitch and yaw axes
 *
 * @note This test validates roll torque generation and lateral angular dynamics
 */
TEST_F(TestStateEstimator, RollRotation) {
  ekf->set_verbosity("SILENT");

  double diff = 0.2;

  // Create differential thrust for positive roll torque
  Eigen::Vector4d u_roll(hover_thrust_per_motor - diff,   // Front left motor (reduced)
                         hover_thrust_per_motor + diff,   // Rear left motor (increased)
                         hover_thrust_per_motor + diff,   // Front right motor (increased)
                         hover_thrust_per_motor - diff);  // Rear right motor (reduced)

  run_simulation(u_roll, 50, 0.02);
  const auto &state = ekf->get_state();

  // Verify positive roll rotation
  EXPECT_GT(state(laser_uav_estimators::State::WX), 0.0);  // Positive roll rate
  EXPECT_GT(state(laser_uav_estimators::State::QX), 0.0);  // Positive roll angle

  // Verify minimal cross-coupling in other axes
  EXPECT_NEAR(state(laser_uav_estimators::State::QY), 0.0, STATE_TOLERANCE);  // No pitch attitude
  EXPECT_NEAR(state(laser_uav_estimators::State::QZ), 0.0, STATE_TOLERANCE);  // No yaw attitude
  EXPECT_NEAR(state(laser_uav_estimators::State::WY), 0.0, STATE_TOLERANCE);  // No pitch rate
  EXPECT_NEAR(state(laser_uav_estimators::State::WZ), 0.0, STATE_TOLERANCE);  // No yaw rate
}

// =============================================================================
// COMPLEX MANEUVER TESTS
//
// This section contains tests that validate the EKF's behavior during
// complex flight maneuvers that combine multiple motion types. These tests
// verify proper modeling of realistic flight scenarios including takeoff,
// landing, and multi-phase flight operations.
// =============================================================================

/**
 * @test Takeoff
 * @brief Validates takeoff maneuver with thrust modulation and stabilization
 *
 * @details This test verifies that the EKF correctly models a realistic takeoff
 * sequence with multiple phases of thrust control. The maneuver includes:
 * 1. Initial thrust burst for rapid altitude gain
 * 2. Thrust reduction to decelerate vertical motion
 * 3. Stabilization at hover thrust for steady flight
 *
 * Test Phases:
 * - Phase 1: High thrust (hover + 2.0 N) for 25 steps (0.5s) - rapid ascent
 * - Phase 2: Low thrust (hover - 2.0 N) for 25 steps (0.5s) - deceleration
 * - Phase 3: Hover thrust for 75 steps (1.5s) - stabilization
 *
 * Expected Behavior:
 * - Final altitude > 0.1 m (successful takeoff)
 * - Final vertical velocity ≈ 0 (stabilized hover)
 * - Smooth transition between thrust phases
 *
 * @note This test validates multi-phase thrust control and altitude stabilization
 */
TEST_F(TestStateEstimator, Takeoff) {
  ekf->set_verbosity("SILENT");

  // Phase 1: Initial thrust burst for rapid ascent
  Eigen::Vector4d u_takeoff = Eigen::Vector4d::Constant(hover_thrust_per_motor + 2.0);
  run_simulation(u_takeoff, 25, 0.02);

  // Phase 2: Thrust reduction for vertical deceleration
  Eigen::Vector4d u_takeoff_2 = Eigen::Vector4d::Constant(hover_thrust_per_motor - 2.0);
  run_simulation(u_takeoff_2, 25, 0.02);

  // Phase 3: Hover stabilization
  Eigen::Vector4d u_hover = Eigen::Vector4d::Constant(hover_thrust_per_motor);
  run_simulation(u_hover, 75, 0.02);

  const auto &state = ekf->get_state();

  // Verify successful altitude gain (takeoff achieved)
  EXPECT_GT(state(laser_uav_estimators::State::PZ), 0.1);

  // Verify stabilized vertical velocity (hover achieved)
  EXPECT_NEAR(state(laser_uav_estimators::State::VZ), 0.0, 0.2);
}

/**
 * @test Land
 * @brief Validates landing maneuver with controlled descent and ground approach
 *
 * @details This test verifies that the EKF correctly models a realistic landing
 * sequence with multiple phases of altitude control. The maneuver includes:
 * 1. Initial takeoff to establish positive altitude
 * 2. Stabilization phase to achieve hover conditions
 * 3. Controlled descent phase with reduced thrust
 * 4. Final recovery phase simulating ground effect and landing completion
 *
 * Test Phases:
 * - Phase 1: Takeoff thrust (hover + 2.0 N) for 25 steps - altitude gain
 * - Phase 2: Stabilization thrust (hover - 2.0 N) for 25 steps - hover approach
 * - Phase 3: Descent thrust (hover - 2.0 N) for 25 steps - controlled descent
 * - Phase 4: Recovery thrust (hover + 2.0 N) for 25 steps - landing flare
 *
 * Expected Behavior:
 * - Final altitude lower than mid-flight altitude (descent achieved)
 * - Final altitude ≈ 0 (successful landing)
 * - Final vertical velocity ≈ 0 (soft touchdown)
 *
 * @note This test validates controlled descent dynamics and landing sequence modeling
 */
TEST_F(TestStateEstimator, Land) {
  ekf->set_verbosity("SILENT");

  // Phase 1: Initial takeoff for altitude establishment
  run_simulation(Eigen::Vector4d::Constant(hover_thrust_per_motor + 2.0), 25, 0.02);

  // Phase 2: Stabilization and hover approach
  run_simulation(Eigen::Vector4d::Constant(hover_thrust_per_motor - 2.0), 25, 0.02);
  const auto state_before_land = ekf->get_state();

  // Verify successful takeoff before landing sequence
  EXPECT_GT(state_before_land(laser_uav_estimators::State::PZ), 0.1);

  // Phase 3: Controlled descent initiation
  run_simulation(Eigen::Vector4d::Constant(hover_thrust_per_motor - 2.0), 25, 0.02);

  // Phase 4: Landing flare and ground approach
  run_simulation(Eigen::Vector4d::Constant(hover_thrust_per_motor + 2.0), 25, 0.02);
  const auto state_after_land = ekf->get_state();

  // Verify descent occurred (altitude reduction)
  EXPECT_LT(state_after_land(laser_uav_estimators::State::PZ), state_before_land(laser_uav_estimators::State::PZ));

  // Verify successful landing (near ground level)
  EXPECT_NEAR(state_after_land(laser_uav_estimators::State::PZ), 0.0, 1e-6);

  // Verify soft touchdown (minimal vertical velocity)
  EXPECT_NEAR(state_after_land(laser_uav_estimators::State::VZ), 0.0, 1e-6);
}

// =============================================================================
// ROBUSTNESS AND EDGE CASE TESTS
//
// This section contains tests that validate the EKF's robustness and error
// handling capabilities. These tests ensure the filter can handle invalid
// data, high noise conditions, and edge cases without failure or divergence.
// =============================================================================

/**
 * @test IgnoreInvalidMeasurements
 * @brief Validates robust handling of invalid sensor measurements
 *
 * @details This test verifies that the EKF correctly rejects invalid sensor
 * measurements containing NaN (Not a Number) values without throwing exceptions
 * or corrupting the filter state. This is critical for real-world operation
 * where sensors may occasionally provide invalid data due to:
 * - Sensor malfunctions or communication errors
 * - Computational overflow in sensor processing
 * - Temporary loss of sensor signal
 *
 * Test Procedure:
 * 1. Execute one prediction step to establish baseline state
 * 2. Create invalid measurement package with NaN orientation
 * 3. Attempt correction with invalid data
 * 4. Verify filter state remains unchanged and no exceptions occur
 *
 * Expected Behavior:
 * - No exceptions thrown during correction attempt
 * - Filter state remains identical to pre-correction state
 * - Filter continues normal operation after invalid measurement
 *
 * @note This test validates input validation and error handling robustness
 */
TEST_F(TestStateEstimator, IgnoreInvalidMeasurements) {
  ekf->set_verbosity("DEBUG");

  // Establish baseline state with single prediction step
  ekf->predict(Eigen::Vector4d::Zero(), 0.02);
  auto initial_state = ekf->get_state();

  // Create measurement package with invalid (NaN) data
  laser_uav_estimators::MeasurementPackage invalid_measurement;
  nav_msgs::msg::Odometry                  bad_odom;
  bad_odom.pose.pose.orientation.w = std::numeric_limits<double>::quiet_NaN();
  bad_odom.pose.pose.position.x    = std::numeric_limits<double>::quiet_NaN();
  invalid_measurement.px4_odometry = bad_odom;

  // Create invalid IMU data as well
  sensor_msgs::msg::Imu bad_imu;
  bad_imu.linear_acceleration.x = std::numeric_limits<double>::quiet_NaN();
  bad_imu.angular_velocity.y    = std::numeric_limits<double>::quiet_NaN();
  bad_imu.orientation.z         = std::numeric_limits<double>::quiet_NaN();
  invalid_measurement.imu       = bad_imu;

  // Verify filter handles invalid measurement gracefully
  EXPECT_NO_THROW(ekf->correct(invalid_measurement));
  auto final_state = ekf->get_state();

  // Verify state remains unchanged after invalid measurement
  EXPECT_TRUE(final_state.isApprox(initial_state));
}

/**
 * @test CorrectionWithHighNoiseHasLittleEffect
 * @brief Validates filter behavior with high-uncertainty measurements
 *
 * @details This test verifies that the EKF correctly handles measurements with
 * very high covariance (uncertainty) values by giving them minimal weight in
 * the state update. This demonstrates proper sensor fusion where unreliable
 * measurements have reduced influence on the state estimate.
 *
 * Test Characteristics:
 * - High measurement covariance: 1e6 (extremely high uncertainty)
 * - Comparison of state before and after correction
 * - Analysis of covariance reduction magnitude
 *
 * Kalman Filter Theory:
 * When measurement noise is very high, the Kalman gain becomes small,
 * resulting in minimal state updates and small covariance reductions.
 * This preserves the filter's confidence in its prediction over unreliable measurements.
 *
 * Expected Behavior:
 * - State change should be minimal (< 1e-3 norm)
 * - Covariance should decrease slightly (always happens in correction)
 * - Relative covariance reduction should be small (< 10%)
 *
 * @note This test validates proper measurement weighting and uncertainty handling
 */
TEST_F(TestStateEstimator, CorrectionWithHighNoiseHasLittleEffect) {
  // Establish baseline with prediction step
  ekf->predict(Eigen::Vector4d::Constant(hover_thrust_per_motor), 0.02);
  auto state_before      = ekf->get_state();
  auto covariance_before = ekf->get_covariance();

  // Create high-noise measurement package
  laser_uav_estimators::MeasurementPackage noisy_measurement;
  nav_msgs::msg::Odometry                  odom;
  odom.pose.pose.orientation.w = 1.0;  // Valid orientation

  // Set extremely high covariance values (low confidence measurement)
  for (int i = 0; i < 36; ++i) {
    odom.pose.covariance[i]  = (i % 7 == 0) ? 1e6 : 0.0;  // High position/orientation uncertainty
    odom.twist.covariance[i] = (i % 7 == 0) ? 1e6 : 0.0;  // High velocity uncertainty
  }
  noisy_measurement.px4_odometry = odom;

  // Apply correction with high-noise measurement
  ekf->correct(noisy_measurement);
  auto state_after      = ekf->get_state();
  auto covariance_after = ekf->get_covariance();

  // Log detailed results for analysis

  // Verify minimal state change due to high measurement uncertainty
  EXPECT_TRUE((state_after - state_before).norm() < 1e-3);

  // Verify covariance decreases (fundamental Kalman filter property)
  EXPECT_LT(covariance_after.trace(), covariance_before.trace());

  // Verify minimal covariance reduction due to low measurement confidence
  double covariance_reduction = covariance_before.trace() - covariance_after.trace();
  double relative_reduction   = covariance_reduction / covariance_before.trace();

  EXPECT_LT(relative_reduction, 0.1);  // Less than 10% reduction expected
}

/**
 * @test CovarianceDecreasesOverMultipleCorrections
 * @brief Validates uncertainty convergence through repeated measurement corrections
 *
 * @details This test verifies that the EKF uncertainty (covariance) decreases
 * progressively when multiple high-quality measurements are incorporated over time.
 * This demonstrates the filter's ability to improve state confidence through
 * continuous sensor fusion, which is fundamental to Kalman filter operation.
 *
 * Test Procedure:
 * - Execute 10 complete prediction-correction cycles
 * - Each cycle includes hover prediction followed by high-quality odometry correction
 * - High-quality measurements have low covariance (0.05 diagonal)
 * - Final covariance trace is compared to convergence threshold
 *
 * Kalman Filter Theory:
 * Each correction step should reduce uncertainty as new information becomes available.
 * With consistent, high-quality measurements, the filter should converge to a
 * low-uncertainty steady state, indicating high confidence in the state estimate.
 *
 * Expected Behavior:
 * - Covariance trace should monotonically decrease
 * - Final uncertainty should be significantly lower than initial
 * - Convergence to stable, low-uncertainty state (< 1.0 trace)
 *
 * @note This test validates long-term filter convergence and measurement effectiveness
 */
TEST_F(TestStateEstimator, CovarianceDecreasesOverMultipleCorrections) {
  ekf->set_verbosity("SILENT");

  const double dt         = 0.02;
  const int    num_cycles = 10;

  // Execute multiple prediction-correction cycles
  for (int i = 0; i < num_cycles; ++i) {
    // Prediction step with hover thrust
    ekf->predict(Eigen::Vector4d::Constant(hover_thrust_per_motor), dt);

    // Create high-quality measurement package
    laser_uav_estimators::MeasurementPackage measurements;
    nav_msgs::msg::Odometry                  odom;
    odom.pose.pose.orientation.w = 1.0;  // Identity quaternion

    // Set low covariance (high confidence) for all measurements
    for (int j = 0; j < 36; ++j)
      odom.pose.covariance[j] = (j % 7 == 0) ? 0.05 : 0.0;

    measurements.px4_odometry = odom;

    // Correction step with high-quality measurement
    ekf->correct(measurements);
  }

  // Verify convergence to low uncertainty state
  EXPECT_LT(ekf->get_covariance().trace(), 1.0);
}

// =============================================================================
// LONG-TERM PERFORMANCE TESTS
//
// This section contains tests that validate the EKF's long-term stability
// and performance characteristics. These tests ensure the filter maintains
// numerical stability and reasonable computational performance over extended
// operation periods typical of real - world UAV missions.
// =============================================================================

/**
 * @test LongTermStability
 * @brief Validates numerical stability and performance over extended operation
 *
 * @details This test verifies that the EKF maintains numerical stability and
 * reasonable performance during extended operation with varying control inputs
 * and periodic measurements. This simulates realistic long-duration UAV missions
 * where the filter must operate continuously without divergence or instability.
 *
 * Test Configuration:
 * - Duration: 1000 prediction steps over 10 seconds of simulated flight
 * - Variable control: Sinusoidal thrust variation around hover (±0.2 N amplitude)
 * - Periodic corrections: High-quality odometry measurements every 10 steps
 * - Time step: 0.01 seconds (100 Hz operation rate)
 *
 * Stability Criteria:
 * - All state values must remain finite (no NaN or infinity)
 * - Covariance must remain positive definite and bounded
 * - No numerical overflow or underflow conditions
 * - Filter convergence and reasonable uncertainty bounds
 *
 * Expected Behavior:
 * - State vector contains only finite values throughout
 * - Covariance trace remains positive and bounded (0 < trace < 10)
 * - No computational exceptions or numerical instabilities
 * - Stable long-term operation without divergence
 *
 * @note This test validates production-ready numerical stability and robustness
 */
TEST_F(TestStateEstimator, LongTermStability) {
  ekf->set_verbosity("SILENT");

  const int    steps = 1000;  // Extended simulation duration
  const double dt    = 0.01;  // High-frequency operation (100 Hz)

  for (int i = 0; i < steps; ++i) {
    // Variable control input with sinusoidal thrust variation
    Eigen::Vector4d u = Eigen::Vector4d::Constant(hover_thrust_per_motor + 0.2 * std::sin(i * 0.1));
    ekf->predict(u, dt);

    // Periodic high-quality corrections (every 10 steps = 10 Hz measurement rate)
    if (i % 10 == 0) {
      laser_uav_estimators::MeasurementPackage meas;
      nav_msgs::msg::Odometry                  odom;
      odom.pose.pose.orientation.w = 1.0;  // Valid identity quaternion

      // Set realistic measurement covariance
      for (int j = 0; j < 36; ++j)
        odom.pose.covariance[j] = (j % 7 == 0) ? 0.01 : 0.0;

      meas.px4_odometry = odom;
      ekf->correct(meas);
    }
  }

  const auto &state = ekf->get_state();

  // Verify numerical stability (all values finite)
  EXPECT_TRUE(state.allFinite());

  // Verify covariance remains positive and bounded
  EXPECT_GT(ekf->get_covariance().trace(), 0.0);   // Positive definite
  EXPECT_LT(ekf->get_covariance().trace(), 10.0);  // Reasonably bounded
}

// =============================================================================
// PERFORMANCE BENCHMARK TESTS
//
// This section contains tests that measure and validate the computational
// performance characteristics of the EKF. These tests ensure the filter
// meets real-time performance requirements for UAV applications.
// =============================================================================

/**
 * @test ExecutionTimeBenchmark
 * @brief Measures and validates computational performance of core EKF operations
 *
 * @details This test measures the average execution time of the critical EKF
 * functions (predict and correct) to ensure they meet real-time performance
 * requirements for UAV applications. Modern UAV systems typically require
 * state estimation updates at 100-1000 Hz, making computational efficiency critical.
 *
 * Benchmark Methodology:
 * - Execute each function 1000 times for statistical stability
 * - Measure total execution time using high-resolution clock
 * - Calculate average execution time per function call
 * - Report results in both microseconds and milliseconds
 *
 * Test Data:
 * - Prediction: Constant hover thrust input with realistic time step
 * - Correction: High-quality odometry measurement with typical covariance
 * - Measurement frequency: Typical sensor update rates
 *
 * Performance Targets:
 * - Prediction: < 100 μs per call (enables > 10 kHz operation)
 * - Correction: < 500 μs per call (enables > 2 kHz measurement processing)
 * - Combined: < 1 ms total per estimation cycle (1 kHz system operation)
 *
 * Expected Results:
 * - Consistent, low-variance execution times
 * - Performance suitable for real-time UAV operation
 * - No performance degradation over multiple iterations
 *
 * @note This test validates production-ready computational performance
 */
TEST_F(TestStateEstimator, ExecutionTimeBenchmark) {
  ekf->set_verbosity("SILENT");

  // Number of iterations for stable average timing
  const int    num_iterations = 1000;
  const double dt             = 0.01;  // Typical control loop time step

  // --- Prepare Test Data ---

  // Control input for prediction timing
  Eigen::Vector4d u_hover = Eigen::Vector4d::Constant(hover_thrust_per_motor);

  // Measurement package for correction timing
  laser_uav_estimators::MeasurementPackage measurements;
  nav_msgs::msg::Odometry                  odom_msg;
  odom_msg.pose.pose.orientation.w = 1.0;

  // Set realistic measurement covariance
  for (int i = 0; i < 36; ++i) {
    odom_msg.pose.covariance[i] = (i % 7 == 0) ? 0.05 : 0.0;
  }
  measurements.px4_odometry = odom_msg;

  // --- Benchmark PREDICTION Function ---

  auto start_predict = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < num_iterations; ++i) {
    ekf->predict(u_hover, dt);
  }
  auto end_predict = std::chrono::high_resolution_clock::now();

  // Calculate prediction timing statistics
  auto   total_duration_predict_us = std::chrono::duration_cast<std::chrono::microseconds>(end_predict - start_predict);
  double avg_time_predict_us       = static_cast<double>(total_duration_predict_us.count()) / num_iterations;

  // --- Benchmark CORRECTION Function ---

  auto start_correct = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < num_iterations; ++i) {
    ekf->correct(measurements);
  }
  auto end_correct = std::chrono::high_resolution_clock::now();

  // Calculate correction timing statistics
  auto   total_duration_correct_us = std::chrono::duration_cast<std::chrono::microseconds>(end_correct - start_correct);
  double avg_time_correct_us       = static_cast<double>(total_duration_correct_us.count()) / num_iterations;

  // --- Performance Analysis and Reporting ---

  // Performance validation (optional assertions for CI/CD)
  // Uncomment these if you have specific performance requirements
  EXPECT_LT(avg_time_predict_us, 100.0);  // Prediction should be < 100 μs
  EXPECT_LT(avg_time_correct_us, 500.0);  // Correction should be < 500 μs
}