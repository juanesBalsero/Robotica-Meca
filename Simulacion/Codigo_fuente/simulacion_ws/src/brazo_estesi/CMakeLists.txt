cmake_minimum_required(VERSION 3.8)
project(brazo_estesi)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch meshes urdf trajectories worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/ejecutar_trayectoria_csv.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

