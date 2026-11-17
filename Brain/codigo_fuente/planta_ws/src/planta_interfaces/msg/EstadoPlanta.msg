# Estado de la planta (/planta/estado), cada 1 s
string estado                 # OPERANDO, FALLO o PARADA_GLOBAL
uint8[5] estados_robots       # 0 esperando, 1 ejecutando, 2 finalizo, 3 error (3 tambien si el robot esta sin comunicacion)
uint32[5] cajas_robots        # caja que tiene cada robot (0 si ninguna)
uint32[5] cajas_en_cola       # cantidad de cajas esperando a cada robot
