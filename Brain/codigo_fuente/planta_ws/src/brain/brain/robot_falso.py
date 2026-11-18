"""Robot de prueba que cumple el contrato del documento (seccion 7.2), sin ROS.

Sirve para probar el brain cuando los grupos reales todavia no estan listos.
No mueve nada: solo "tarda" `duracion` segundos en cada etapa.
"""
from brain.brain_core import ERROR, EJECUTANDO, ESPERANDO, FINALIZO, MEDICAMENTOS


class RobotFalso:
    def __init__(self, robot_id, duracion=5.0, retardo_libre=0.5, t_hb=3.0):
        self.id = robot_id
        self.duracion = duracion
        self.retardo_libre = retardo_libre    # entre "finalizo" y "esperando"
        self.t_hb = t_hb                      # sin latido del brain = parada segura
        self.estado = ESPERANDO
        self.caja_id = 0
        self.seq = 0
        self.pausado = False
        self.estop = False
        self.restante = 0.0
        self.t_libre = None
        self.t_ultimo_latido = None
        self.publicar = False                 # el nodo publica el estado cuando esto es True
        # Inyeccion de fallos (se pueden cambiar en caliente)
        self.mudo = False                     # ignora todas las ordenes (no confirma)
        self.error_en_caja = 0                # reporta error al ejecutar esta caja

    def mensaje(self):
        return self.id, self.estado, self.caja_id, self.seq

    def on_cmd(self, tarea, caja_id, seq, medicamento):
        if self.mudo:
            return
        if tarea == f'T{self.id}':
            if seq == self.seq and self.estado != ESPERANDO:   # reintento: solo reconfirma
                self.publicar = True
                return
            self.seq, self.caja_id = seq, caja_id
            if self.id == 2 and medicamento not in MEDICAMENTOS:
                self.estado = ERROR                              # proteccion del robot 2
            else:
                self.estado = EJECUTANDO
                self.restante = self.duracion
            self.publicar = True
        elif tarea == 'PAUSAR':
            self.pausado = True
        elif tarea == 'REANUDAR':
            if not self.estop:
                self.pausado = False
        elif tarea == 'ABORTAR':
            self.estado, self.caja_id, self.restante, self.t_libre = ESPERANDO, 0, 0.0, None
            self.pausado = False
            self.publicar = True

    def on_estop(self, activo):
        self.estop = activo
        if activo:
            self.pausado = True   # al liberar, sigue parado hasta recibir REANUDAR

    def on_latido(self, now):
        self.t_ultimo_latido = now

    def tick(self, dt, now):
        if self.t_ultimo_latido is not None and now - self.t_ultimo_latido > self.t_hb:
            self.pausado = True   # parada segura: el brain cayo
        if self.pausado:
            return
        if self.estado == EJECUTANDO:
            self.restante -= dt
            if self.error_en_caja and self.error_en_caja == self.caja_id and self.restante <= self.duracion / 2:
                self.estado = ERROR
                self.publicar = True
            elif self.restante <= 0:
                self.estado, self.t_libre = FINALIZO, self.retardo_libre
                self.publicar = True
        elif self.estado == FINALIZO and self.t_libre is not None:
            self.t_libre -= dt
            if self.t_libre <= 0:
                self.estado, self.caja_id, self.t_libre = ESPERANDO, 0, None
                self.publicar = True
