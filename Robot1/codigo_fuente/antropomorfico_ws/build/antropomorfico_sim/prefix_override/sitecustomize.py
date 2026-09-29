import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/juanes/Robotica-Meca/Robot1/codigo_fuente/antropomorfico_ws/install/antropomorfico_sim'
