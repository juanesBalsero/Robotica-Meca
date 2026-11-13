# Punto de la trayectoria deseada (lo publica trajectory_publisher)
std_msgs/Header header
float64[3] position   # x_d  [m]
float64[3] velocity   # x_d punto [m/s]
bool gripper          # true = garra cerrada, false = abierta
