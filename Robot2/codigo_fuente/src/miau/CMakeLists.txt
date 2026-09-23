cmake_minimum_required(VERSION 3.8)
project(miau)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_executable(nodo src/nodo.cpp)
ament_target_dependencies(nodo
  rclcpp std_msgs sensor_msgs kdl_parser orocos_kdl ament_index_cpp)

add_executable(vti2 src/vti2.cpp)
ament_target_dependencies(vti2
  rclcpp std_msgs sensor_msgs kdl_parser orocos_kdl ament_index_cpp)

install(TARGETS nodo vti2 DESTINATION lib/${PROJECT_NAME})
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
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