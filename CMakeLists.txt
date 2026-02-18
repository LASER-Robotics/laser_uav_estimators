cmake_minimum_required(VERSION 3.8)
project(laser_uav_estimators)

# =====================================================
# C++ standard
# =====================================================
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =====================================================
# Dependencies
# =====================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(laser_uav_lib REQUIRED)
find_package(autodiff REQUIRED)
find_package(Eigen3 REQUIRED)

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

# =====================================================
# Library
# =====================================================
add_library(laser_uav_estimators SHARED
  src/eskf_state_estimator.cpp
  src/mekf_state_estimator.cpp
)

target_include_directories(laser_uav_estimators PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(laser_uav_estimators
  Eigen3::Eigen
  ${autodiff_LIBRARIES}
)

ament_target_dependencies(laser_uav_estimators ${dependencies})

# =====================================================
# Install
# =====================================================
install(
  TARGETS laser_uav_estimators
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  DIRECTORY params
  DESTINATION share/${PROJECT_NAME}/
)

# =====================================================
# Export
# =====================================================
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})
ament_export_include_directories(include)
ament_export_libraries(laser_uav_estimators)

# =====================================================
# Tests
# =====================================================
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()

endif()

ament_package()
