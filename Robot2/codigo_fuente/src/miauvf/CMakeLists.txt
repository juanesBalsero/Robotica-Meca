cmake_minimum_required(VERSION 3.8)
project(miau)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_executable(nodo_jacobiano src/nodo_jacobiano.cpp)
target_include_directories(nodo_jacobiano PRIVATE include)
ament_target_dependencies(nodo_jacobiano
  rclcpp std_msgs sensor_msgs kdl_parser orocos_kdl ament_index_cpp)

add_executable(nodo_cinematico src/nodo_cinematico.cpp)
target_include_directories(nodo_cinematico PRIVATE include)
ament_target_dependencies(nodo_cinematico
  rclcpp std_msgs sensor_msgs kdl_parser orocos_kdl ament_index_cpp)

install(TARGETS nodo_jacobiano nodo_cinematico DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/gui_miau.py scripts/esp32_robot_bridge.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY
    urdf
    launch
    meshes
    rviz
    config
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
