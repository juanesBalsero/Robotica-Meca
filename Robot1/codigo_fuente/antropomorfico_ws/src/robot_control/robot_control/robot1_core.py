"""Lógica del Robot 1 según el contrato de la planta (sin dependencias de ROS).

Traduce las órdenes del brain (T1, PAUSAR, REANUDAR, ABORTAR, parada global,
latido) a llamadas de los servicios de `trajectory_publisher` (~/start, ~/stop,
~/reset) y deduce el estado del robot (0 esperando, 1 ejecutando, 2 finalizó,
3 error) de lo que ya publican los nodos existentes:

  /trajectory_publisher/estado   espera | corriendo | pausa | fin | volviendo
  /joint_states                  llega mientras el enlace con la ESP32 está vivo
  /tracking_error                error de seguimiento (norma, en metros)

Todos los métodos reciben `now` (segundos, reloj monotónico).
"""
from collections import namedtuple
from dataclasses import dataclass

ESPERANDO, EJECUTANDO, FINALIZO, ERROR = 0, 1, 2, 3

# tipo: 'srv' (dato = 'start' | 'stop' | 'reset'), 'publicar' o 'log'
Salida = namedtuple('Salida', ['tipo', 'dato'])


@dataclass
class Parametros:
    e_inicio_m: float = 0.02        # error máximo para considerar al robot "en home / armado"
    e_error_m: float = 0.12         # error de seguimiento a partir del cual se reporta error
    t_link_s: float = 1.0           # sin /joint_states (o /tracking_error) por más de esto = enlace caído
    t_traj_s: float = 3.0           # sin estado de trajectory_publisher por más de esto = nodo caído
    t_hb_s: float = 3.0             # sin latido del brain por más de esto = parada segura
    t_arranque_s: float = 2.0       # tiempo máximo para ver 'corriendo' tras llamar ~/start
    t_min_finalizo_s: float = 0.5   # mínimo en estado 2 antes de pasar a 0
    t_asentar_s: float = 15.0       # tiempo máximo para quedar en home (tras terminar o abortar)


class Robot1Core:
    ROBOT_ID = 1
    REINTENTO_S = 0.5               # cada cuánto se repite un start/stop que no tuvo efecto

    def __init__(self, params, now):
        self.p = params
        self.estado = ESPERANDO
        self.caja_id = 0
        self.seq = 0
        self.pausado = False
        self.estop = False
        # Lo último que se supo de los demás nodos
        self.t_js = self.t_traj = self.t_err = self.t_latido = None
        self.err = None
        self.traj = None
        self.sin_latido = False
        # Seguimiento de la tarea
        self.inicio_pendiente = False   # orden aceptada estando en pausa: falta llamar ~/start
        self.vio_corriendo = False
        self.t_arranque = 0.0
        self.t_fin = 0.0
        self.abortando = False
        self.reset_enviado = False
        self.t_abort = 0.0
        self._t_stop = self._t_start = -1e9
        self._salidas = []

    # ------------------------------------------------------------------ utilidades
    def retirar_salidas(self):
        s, self._salidas = self._salidas, []
        return s

    def _log(self, texto):
        self._salidas.append(Salida('log', texto))

    def _pub(self):
        self._salidas.append(Salida('publicar', None))

    def _srv(self, nombre):
        self._salidas.append(Salida('srv', nombre))

    def listo(self):
        """Ya se vieron el enlace con la ESP32 y trajectory_publisher: recién ahí se publica estado."""
        return self.t_js is not None and self.t_traj is not None

    def mensaje(self):
        """(robot_id, estado, caja_id, seq) o None si todavía no se presenta."""
        return (self.ROBOT_ID, self.estado, self.caja_id, self.seq) if self.listo() else None

    @staticmethod
    def _fresco(t, now, limite):
        return t is not None and now - t <= limite

    def _enlace_ok(self, now):
        return self._fresco(self.t_js, now, self.p.t_link_s)

    def _traj_vivo(self, now):
        return self._fresco(self.t_traj, now, self.p.t_traj_s)

    def _armado(self, now):
        return self._fresco(self.t_err, now, self.p.t_link_s) and self.err <= self.p.e_inicio_m

    def _no_listo(self, now):
        if not self._enlace_ok(now):
            return 'sin datos de la ESP32 (/joint_states)'
        if not self._traj_vivo(now):
            return 'trajectory_publisher no responde'
        if self.traj not in ('espera', 'fin'):
            return f'trajectory_publisher esta en {self.traj}'
        if not self._fresco(self.t_err, now, self.p.t_link_s):
            return 'sin /tracking_error'
        if self.err > self.p.e_inicio_m:
            return f'robot fuera de home (error {self.err:.3f} m)'
        return None

    # ------------------------------------------------------------------ entradas de los otros nodos
    def on_joint_states(self, now):
        self.t_js = now

    def on_tracking_error(self, norma, now):
        self.err, self.t_err = norma, now

    def on_traj_estado(self, estado, now):
        self.traj, self.t_traj = estado, now
        if estado == 'corriendo':
            self.vio_corriendo = True

    def on_latido(self, now):
        self.t_latido = now
        if self.sin_latido:
            self.sin_latido = False
            self._log('Latido del brain recuperado')
            if self.estado != EJECUTANDO and not self.estop:
                self.pausado = False   # sin tarea que proteger; con tarea espera REANUDAR

    def on_estop(self, activo, now):
        self.estop = activo
        if activo:
            self._pausar('parada global')
        else:
            self._log('Parada global liberada: sigue en pausa hasta recibir REANUDAR')

    def on_srv_resultado(self, nombre, ok, mensaje, now):
        if ok or nombre == 'stop':       # "No esta corriendo" es inofensivo
            return
        if nombre == 'start':
            if 'corriendo' in mensaje.lower():
                return
            if self.estado == EJECUTANDO:
                self._error(f'no se pudo iniciar la trayectoria: {mensaje}')
        elif nombre == 'reset':
            if 'no disponible' in mensaje.lower():
                self._error(f'no se pudo volver al inicio: {mensaje}')
            else:
                self.reset_enviado = False   # se reintenta; t_asentar_s pone el límite

    # ------------------------------------------------------------------ órdenes del brain
    def on_cmd(self, tarea, caja_id, seq, now):
        if tarea == 'T1':
            self._cmd_t1(caja_id, seq, now)
        elif tarea == 'PAUSAR':
            self._pausar('PAUSAR')
        elif tarea == 'REANUDAR':
            if self.estop:
                self._log('REANUDAR ignorado: la parada global sigue activa')
                return
            self.pausado = False
            self._log('REANUDAR')
        elif tarea == 'ABORTAR':
            self._abortar(now)
        # T2..T5 u otras: no son de este robot

    def _cmd_t1(self, caja_id, seq, now):
        if seq <= self.seq:
            if seq == self.seq:
                self._pub()              # reintento: no repite la tarea, reconfirma el estado
            return                       # duplicado viejo: se ignora
        if self.estop or self.abortando or self.estado != ESPERANDO:
            self._log(f'T1 (caja {caja_id}) ignorada: estop={self.estop}, abortando={self.abortando}, '
                      f'estado={self.estado}')
            return
        self.seq, self.caja_id = seq, caja_id
        motivo = self._no_listo(now)
        if motivo:
            self._error(f'no puede iniciar la caja {caja_id}: {motivo}')
            return
        self.estado = EJECUTANDO
        self.vio_corriendo = False
        self._pub()                      # estado 1 de inmediato: es la confirmación
        self._log(f'Caja {caja_id} aceptada (seq {seq})')
        if self.pausado:
            self.inicio_pendiente = True
        else:
            self._iniciar(now)

    def _iniciar(self, now):
        self.inicio_pendiente = False
        self.t_arranque = now
        self.vio_corriendo = False
        self._t_start = now
        self._srv('start')

    def _pausar(self, motivo):
        if not self.pausado:
            self._log(f'Pausa: {motivo}')
        self.pausado = True

    def _abortar(self, now):
        self.abortando = True
        self.pausado = False
        self.inicio_pendiente = False
        self.reset_enviado = False
        self.t_abort = now
        self._log('ABORTAR: detener, volver al inicio y esperar a quedar listo')

    def _error(self, causa):
        if self.estado != ERROR:
            self._log(f'ERROR: {causa}')
            self.estado = ERROR
            self._pub()
        self.inicio_pendiente = False
        self.abortando = False
        if self.traj == 'corriendo':
            self._srv('stop')            # mantiene la posición

    # ------------------------------------------------------------------ ciclo periódico
    def tick(self, now):
        if not self.listo():
            return
        if (self.t_latido is not None and not self.sin_latido
                and now - self.t_latido > self.p.t_hb_s):
            self.sin_latido = True
            self._pausar('el brain dejo de enviar latido: parada segura')

        if self.abortando:
            self._tick_abortar(now)
        elif self.estado != ERROR:
            self._revisar_salud(now)

        if self.abortando:
            return
        # Hacer cumplir la pausa / la reanudación sobre trajectory_publisher
        if self.pausado and self.traj == 'corriendo' and now - self._t_stop > self.REINTENTO_S:
            self._t_stop = now
            self._srv('stop')
        if self.estado == EJECUTANDO and not self.pausado:
            if self.inicio_pendiente:
                self._iniciar(now)
            elif self.traj == 'pausa' and now - self._t_start > self.REINTENTO_S:
                self._t_start = now
                self._srv('start')       # retoma donde quedó
        self._avanzar_tarea(now)

    def _revisar_salud(self, now):
        if not self._enlace_ok(now):
            self._error('se perdio el enlace con la ESP32')
        elif not self._traj_vivo(now):
            self._error('trajectory_publisher dejo de publicar su estado')
        elif (self.estado == EJECUTANDO and self._fresco(self.t_err, now, self.p.t_link_s)
              and self.err > self.p.e_error_m):
            self._error(f'error de seguimiento {self.err:.3f} m')

    def _avanzar_tarea(self, now):
        if self.estado == EJECUTANDO and not self.pausado and self.estado != ERROR:
            if not self.vio_corriendo and not self.inicio_pendiente \
                    and now - self.t_arranque > self.p.t_arranque_s:
                self._error('la trayectoria no arranco')
            elif self.vio_corriendo and self.traj == 'fin':
                self.estado = FINALIZO
                self.t_fin = now
                self._pub()
                self._log(f'Caja {self.caja_id}: etapa terminada')
            elif self.vio_corriendo and self.traj == 'espera':
                self._error('la trayectoria se reinicio por fuera')
        elif self.estado == FINALIZO:
            en_home = self.traj in ('fin', 'espera') and self._armado(now)
            if en_home and now - self.t_fin >= self.p.t_min_finalizo_s:
                self.estado, self.caja_id = ESPERANDO, 0
                self._pub()
            elif now - self.t_fin > self.p.t_asentar_s:
                self._error('no volvio a home al terminar la etapa')

    def _tick_abortar(self, now):
        if self.traj == 'corriendo':
            if now - self._t_stop > self.REINTENTO_S:
                self._t_stop = now
                self._srv('stop')
        elif not self.reset_enviado and self.traj in ('pausa', 'fin', 'espera'):
            self.reset_enviado = True
            self._srv('reset')
        if self.reset_enviado and self.traj == 'espera':
            if self._armado(now):
                self.abortando = False
                self.estado, self.caja_id = ESPERANDO, 0
                self._pub()
                self._log('ABORTAR terminado: robot en home y esperando')
                return
        if now - self.t_abort > self.p.t_asentar_s:
            self._error('no quedo en home tras ABORTAR (revisar el robot y reiniciar kinematic_control)')
