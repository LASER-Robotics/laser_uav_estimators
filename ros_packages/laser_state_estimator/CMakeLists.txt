cmake_minimum_required(VERSION 3.8)
project(laser_state_estimator)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

set(dependencies
rclcpp 
rclcpp_lifecycle 
rclcpp_components
nav_msgs
tf2_geometry_msgs
Eigen3
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

set(executable_quadrotor_model quadrotor_model)
set(library_quadrotor_model quadrotor_model_core)

add_executable(${executable_quadrotor_model}
  src/quadrotor_model_main.cpp
)

add_library(${library_quadrotor_model} SHARED
  src/quadrotor_model_node.cpp
) 

ament_target_dependencies(${library_quadrotor_model}
  ${dependencies}
)

target_link_libraries(${executable_quadrotor_model}
  ${library_quadrotor_model}
)

target_include_directories(${library_quadrotor_model} PRIVATE src/include)

rclcpp_components_register_nodes(${library_quadrotor_model} "laser_state_estimator::QuadrotorModel")

install(TARGETS ${library_quadrotor_model}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_quadrotor_model}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  params
  launch
	DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_package()
