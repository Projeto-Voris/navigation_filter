cmake_minimum_required(VERSION 3.8)
project(sensor_fusion_modelling)

# 1. Opções do Compilador
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 2. Dependências do ROS 2 e Externas
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(dvl_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

# 3. Criar a Biblioteca (onde estão seus modelos e lógica)
add_library(nav_filter_lib
  navigation_filter_library/src/nav_filter.cpp
  navigation_filter_library/src/imu_model.cpp
  navigation_filter_library/src/twist_model.cpp
  navigation_filter_library/src/pose_model.cpp
  navigation_filter_library/src/sensor_model.cpp
  navigation_filter_library/src/error_state.cpp
  navigation_filter_library/src/proccess_model.cpp
  navigation_filter_library/src/state.cpp
)

# Definir onde estão os headers (.hpp) para a biblioteca
target_include_directories(nav_filter_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/navigation_filter_library/include>
  $<INSTALL_INTERFACE:include>
)

# Linkar o Eigen e dependências na biblioteca
target_link_libraries(nav_filter_lib Eigen3::Eigen)
ament_target_dependencies(nav_filter_lib 
  rclcpp 
  sensor_msgs 
  nav_msgs 
  geometry_msgs
  dvl_msgs
)

# 4. Criar o Executável (o Nó do ROS)
# Note que aqui só listamos o arquivo que contém a função main()
add_executable(sensor_fusion_node src/sensor_fusion_node.cpp)

# Linkar a sua biblioteca e as dependências do ROS no executável
target_link_libraries(nav_filter_node nav_filter_lib)

# IMPORTANTE: O nome aqui deve ser exatamente o mesmo do add_executable
ament_target_dependencies(sensor_fusion_node
  rclcpp
  rclcpp_lifecycle
  std_msgs
  dvl_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  lifecycle_msgs
  tf2_ros
)

# 5. Instalação (essencial para o colcon build funcionar)
install(TARGETS
  nav_filter_lib
  nav_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include
  params
  src
  DESTINATION share/${PROJECT_NAME}
)

# 6. Testes (Opcional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()