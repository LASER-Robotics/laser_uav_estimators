cmake_minimum_required(VERSION 3.8)
project(laser_state_estimator)

# --------------------------------------------------------------
# |                     Configuração do Build                    |
# --------------------------------------------------------------

# Configuração do padrão C++
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# Opções de compilação para GCC/Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --------------------------------------------------------------
# |                       Dependências                         |
# --------------------------------------------------------------

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(laser_msgs REQUIRED)
find_package(laser_uav_lib REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}    
  ${laser_uav_lib_INCLUDE_DIRS}                       
)


# --------------------------------------------------------------
# |                   Alvos de Compilação                      |
# --------------------------------------------------------------

# Define variáveis para os nomes dos alvos
set(estimation_library_name "state_estimator_node")
set(estimation_executable_name "state_estimator_main")

# Biblioteca que contém a lógica da classe StateEstimator
add_library(${estimation_library_name} SHARED
  src/state_estimator_node.cpp
)

# Define as dependências da biblioteca (pacotes externos)
ament_target_dependencies(${estimation_library_name}
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  laser_msgs
  # A dependência do laser_uav_lib será herdada através do target_link_libraries
)

# Vincula as bibliotecas da laser_uav_lib à nossa biblioteca
target_link_libraries(${estimation_library_name}
  laser_uav_lib::drone_ekf
)

# Executável principal que contém a função main()
add_executable(${estimation_executable_name}
  src/state_estimator_main.cpp
)

# O executável principal depende apenas do rclcpp
ament_target_dependencies(${estimation_executable_name}
  rclcpp
)

# Vincula a nossa biblioteca de lógica ao executável principal
target_link_libraries(${estimation_executable_name}
  ${estimation_library_name}
)

# Regista o nó como um componente carregável
rclcpp_components_register_nodes(${estimation_library_name} "laser_state_estimator::StateEstimator")


# --------------------------------------------------------------
# |                        Instalação                          |
# --------------------------------------------------------------

# Instala a biblioteca
install(TARGETS
  ${estimation_library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Instala o executável principal
install(TARGETS
  ${estimation_executable_name}
  DESTINATION lib/${PROJECT_NAME}
)

# Instala os ficheiros de cabeçalho
install(DIRECTORY include/
  DESTINATION include/
)

# Instala os ficheiros de lançamento e parâmetros
install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)

# --------------------------------------------------------------
# |                          Testes                            |
# --------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# --------------------------------------------------------------

ament_package()