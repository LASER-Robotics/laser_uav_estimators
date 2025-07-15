cmake_minimum_required(VERSION 3.8)
project(laser_estimation_manager)

# Configuração do padrão C++
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# Opções de compilação para GCC/Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ===== Dependências =====
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(laser_uav_lib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(laser_msgs REQUIRED)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  std_srvs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  sensor_msgs
  Eigen3
  laser_msgs
)

# Inclui diretórios
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${laser_uav_lib_INCLUDE_DIRS}
)

# ============ EstimationManager ============

# Executável principal
set(estimation_executable_name estimation_manager)
add_executable(${estimation_executable_name}
  src/estimation_manager.cpp
)

# Biblioteca do nó (opcional — útil se for usar como componente)
set(estimation_library_name estimation_manager_node)
add_library(${estimation_library_name} SHARED
  src/estimation_manager_node.cpp
)

# Dependências do executável e biblioteca
ament_target_dependencies(${estimation_executable_name}
  ${dependencies}
)

ament_target_dependencies(${estimation_library_name}
  ${dependencies}
)

# Vincula a biblioteca ao executável
target_link_libraries(${estimation_executable_name}
  ${estimation_library_name}
  # laser_uav_lib::attitude_converter
)

# Registrar componente (caso o node use rclcpp_components)
rclcpp_components_register_nodes(${estimation_library_name}
  "laser_estimation_manager::EstimationManager"
)

# ============ Instalação ============

# Instala os executáveis
install(TARGETS
  ${estimation_executable_name}
  DESTINATION lib/${PROJECT_NAME}
)

# Instala a biblioteca
install(TARGETS
  ${estimation_library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Instala os headers
install(DIRECTORY include/
  DESTINATION include/
)

# Instala launch e params se existirem
install(DIRECTORY
  launch
  params
  DESTINATION share/${PROJECT_NAME}
)

# ============ Testes (opcional) ============

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Finaliza o pacote
ament_package()
# ==================== Fim do CMakeLists.txt ====================