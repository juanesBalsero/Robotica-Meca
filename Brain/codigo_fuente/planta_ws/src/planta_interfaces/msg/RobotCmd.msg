# Orden del brain a un robot (/robot_N/cmd)
uint8 robot_id
uint32 caja_id
string tarea          # T1..T5 (la de ese robot), PAUSAR, REANUDAR o ABORTAR
string medicamento    # AA, AB o BB; solo lo usa el robot 2, los demas lo reciben vacio
uint32 seq            # numero que genera el brain; el robot lo repite en su estado
