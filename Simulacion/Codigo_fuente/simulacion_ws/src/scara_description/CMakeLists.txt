cmake_minimum_required(VERSION 3.8)
project(scara_description)
find_package(ament_cmake REQUIRED)
install(DIRECTORY urdf meshes launch config worlds matlab DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/send_trajectory.py
  scripts/log_joint_states.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
