import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/david-romero/Robotica-Meca/Robot4/Codigo fuente/robotica/src/scara_cajas/install/scara_cajas'
