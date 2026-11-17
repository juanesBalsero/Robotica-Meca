cmake_minimum_required(VERSION 3.8)
project(planta_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotCmd.msg"
  "msg/RobotEstado.msg"
  "msg/Progreso.msg"
  "msg/Alerta.msg"
  "msg/EstadoPlanta.msg"
  "srv/IniciarCaja.srv"
)

ament_package()
