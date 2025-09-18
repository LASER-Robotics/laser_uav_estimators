# =============================================================================
# @file CMakeLists.txt
# @brief Build configuration for the laser_uav_estimators package
# @author Wagner Dantas Garcia / Laser UAV Team
# @date September 16, 2025
# @copyright Apache License 2.0
# 
# @details This CMakeLists.txt file defines the build configuration for the 
# laser_uav_estimators ROS2 package. The package provides state estimation
# capabilities for UAV systems using Extended Kalman Filtering with multiple
# sensor fusion including VIO, LIO, IMU, and GPS data sources.
# 
# Key Components:
# - StateEstimator: Main estimation node with lifecycle management
# - Multi-sensor fusion capabilities
# - Integration with laser_uav_lib for filtering and attitude conversion
# - Comprehensive testing framework
# 
# Dependencies:
# - ROS2 Humble or later
# - laser_uav_lib: Core UAV functionality library
# - autodiff: Automatic differentiation for Jacobian calculations
# - Eigen3: Linear algebra operations
# 
# @note This package follows ROS2 ament_cmake build system conventions
# @see https://docs.ros.org/en/humble/How-To-Guides/Ament-CMake-Documentation.html
# =============================================================================

# =============================================================================
# Project Configuration
# 
# Sets the minimum CMake version required and declares the project name.
# The project name is used throughout the build system for naming conventions
# and package identification.
# =============================================================================
cmake_minimum_required(VERSION 3.8)
project(laser_uav_estimators)

# =============================================================================
# C++ Standard Configuration and Compiler Options
# 
# Sets the C++ standard to C++17 which is required for:
# - Modern C++ features used in the codebase
# - Compatibility with ROS2 and its dependencies
# - Advanced template metaprogramming in autodiff library
# 
# Compiler warnings are enabled to catch potential issues early in development.
# =============================================================================

# Set C++17 standard if not already specified
# C++17 is required for advanced template features and ROS2 compatibility
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# Enable comprehensive compiler warnings for code quality
# -Wall: Enable all standard warnings
# -Wextra: Enable additional warnings beyond -Wall
# -Wpedantic: Enforce strict ISO C++ compliance
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =============================================================================
# Dependency Discovery and Configuration
# 
# Locates and configures all required dependencies for the package.
# Each dependency serves a specific purpose in the state estimation system:
# 
# Core ROS2 Dependencies:
# - ament_cmake: Build system integration
# - rclcpp: C++ client library for node creation and communication
# - rclcpp_lifecycle: Lifecycle management for controlled node states
# - rclcpp_components: Component-based node architecture
# 
# Message Dependencies:
# - geometry_msgs: Pose, twist, and spatial relationship messages
# - nav_msgs: Navigation-specific messages (odometry, paths)
# - sensor_msgs: Sensor data messages (IMU, GPS, camera)
# - tf2_ros: Transform library for coordinate frame management
# 
# Custom Dependencies:
# - laser_uav_lib: LASER UAV core functionality (filtering, attitude conversion)
# - autodiff: Automatic differentiation for EKF Jacobian calculations
# - Eigen3: Linear algebra operations for mathematical computations
# =============================================================================

# Find required ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Find custom LASER UAV library
find_package(laser_uav_lib REQUIRED)

# Debug information for laser_uav_lib integration
# Helps troubleshoot dependency issues during development
message(STATUS "laser_uav_lib_FOUND: ${laser_uav_lib_FOUND}")
message(STATUS "laser_uav_lib_INCLUDE_DIRS: ${laser_uav_lib_INCLUDE_DIRS}")
message(STATUS "laser_uav_lib_LIBRARIES: ${laser_uav_lib_LIBRARIES}")

# Find mathematical libraries
find_package(autodiff REQUIRED)  # Automatic differentiation for Jacobians
find_package(Eigen3 REQUIRED)    # Linear algebra operations

# Consolidate ROS2 dependencies for easier management
# This list is used throughout the build configuration
set(dependencies
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2_ros
  laser_uav_lib
)

# =============================================================================
# Main Library Definition
# 
# Creates the primary shared library for the laser_uav_estimators package.
# The library contains the StateEstimator class which implements:
# - Extended Kalman Filter for multi-sensor state estimation
# - Lifecycle node management for controlled operation
# - Integration with various odometry sources (VIO, LIO, PX4)
# - IMU and GPS sensor fusion capabilities
# 
# Library Configuration:
# - SHARED: Creates a shared library (.so) for dynamic linking
# - PUBLIC interface: Exposes headers for dependent packages
# - Mixed linking: Uses both target_link_libraries and ament_target_dependencies
# =============================================================================

# Create the main estimation library
# Library name matches the package name for consistency
add_library(laser_uav_estimators SHARED
  src/state_estimator.cpp  # Main StateEstimator implementation
)

# Configure include directories for the library
# BUILD_INTERFACE: Available during build of this package
# INSTALL_INTERFACE: Available to packages that depend on this one
target_include_directories(laser_uav_estimators PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link against mathematical libraries first
# These libraries use traditional CMake linking approach
# Must be called before ament_target_dependencies to avoid signature conflicts
target_link_libraries(laser_uav_estimators
  ${autodiff_LIBRARIES}  # Automatic differentiation library
  Eigen3::Eigen          # Linear algebra library
)

# Link against ROS2 dependencies using ament conventions
# ament_target_dependencies handles ROS2-specific linking requirements
ament_target_dependencies(laser_uav_estimators ${dependencies})

# =============================================================================
# Installation Configuration
# 
# Defines what gets installed and how other packages can find this one.
# The installation process includes:
# - Library files: The compiled shared library
# - Header files: Public interface for dependent packages
# - Configuration files: Parameter files for runtime configuration
# - Export information: CMake targets for dependent packages
# 
# Installation Destinations:
# - lib/: Compiled libraries and executables
# - include/: Public header files
# - share/: Package-specific data files (parameters, launch files)
# =============================================================================

# Install the main library and create export target
# export_${PROJECT_NAME}: Allows other packages to import this library
install(
  TARGETS laser_uav_estimators  # Install the main library
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib      # Static libraries (.a files)
  LIBRARY DESTINATION lib      # Shared libraries (.so files)
  RUNTIME DESTINATION bin      # Executables (if any)
)

# Install public header files
# Makes headers available to packages that depend on this one
install(
  DIRECTORY include/
  DESTINATION include/
)

# Install parameter configuration files
# Runtime configuration files accessible via share directory
install(
  DIRECTORY params
  DESTINATION share/${PROJECT_NAME}/
)

# =============================================================================
# Package Export Configuration
# 
# Configures how other ROS2 packages can find and use this package.
# Export functions generate the necessary CMake files for:
# - Target importing: Other packages can link against our library
# - Dependency propagation: Transitive dependencies are handled automatically
# - Include directory exposure: Headers are automatically available
# - Library name exposure: Simplified linking for dependent packages
# =============================================================================

# Export targets for other packages to import
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

# Export dependencies so they're automatically found by dependent packages
ament_export_dependencies(${dependencies})

# Export include directories for header file access
ament_export_include_directories(include)

# Export library name for simplified linking in dependent packages
# Allows other packages to use: target_link_libraries(target laser_uav_estimators)
ament_export_libraries(laser_uav_estimators)

# =============================================================================
# Testing Configuration
# 
# Configures the testing framework for the package. Testing is enabled when
# BUILD_TESTING is set to ON (default in development builds).
# 
# Testing Components:
# - GTest Framework: Unit testing for C++ code
# - Linting Tools: Code quality and style checking
# - Custom Tests: Package-specific functionality tests
# 
# Test Structure:
# - test_state_estimator: Unit tests for the StateEstimator class
# - Automatic linting: Code style and quality validation
# 
# The tests verify:
# - EKF mathematical correctness
# - Sensor fusion functionality
# - Lifecycle management behavior
# - Integration with laser_uav_lib components
# =============================================================================

# Enable testing configuration if BUILD_TESTING is ON
if(BUILD_TESTING)
  # Find testing dependencies
  find_package(ament_cmake_gtest REQUIRED)  # GTest framework integration
  find_package(ament_lint_auto REQUIRED)    # Automatic linting tools
  
  # Automatically find and configure linting test dependencies
  # Includes: cpplint, uncrustify, flake8, etc.
  ament_lint_auto_find_test_dependencies()

  # Create unit test executable for StateEstimator
  # Tests the core functionality of the state estimation system
  ament_add_gtest(test_state_estimator test/test_state_estimator.cpp)
  
  # Link test executable against the main library
  # This allows testing of internal library functions and classes
  target_link_libraries(test_state_estimator 
    laser_uav_estimators  # Main library containing StateEstimator
  )
  
  # Ensure test has access to all ROS2 dependencies
  # Required for testing ROS2-specific functionality
  ament_target_dependencies(test_state_estimator ${dependencies})
endif()

# =============================================================================
# Package Finalization
# 
# Mandatory macro call that finalizes the package configuration.
# This must be the last command in the CMakeLists.txt file.
# 
# What ament_package() does:
# - Registers the package with the ROS2 build system
# - Generates package configuration files
# - Validates package setup and dependencies
# - Enables integration with colcon build tool
# =============================================================================

# Finalize package configuration - MUST be last command
ament_package()
