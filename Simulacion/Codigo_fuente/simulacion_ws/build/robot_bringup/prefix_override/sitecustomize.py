import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/juanes/Robotica-Meca/Simulacion/Codigo_fuente/simulacion_ws/install/robot_bringup'
