"""Logica del brain, sin dependencias de ROS.

Aqui vive toda la coordinacion: cajas, robots, reintentos, fallos, parada
global y progreso. El nodo ROS (brain_node.py) solo traduce mensajes a
llamadas de esta clase y publica lo que ella deja en la lista de salidas.
Al no depender de ROS se puede probar con pytest y un reloj simulado.

Todos los metodos reciben `now` (segundos, reloj monotonico).
"""
from collections import deque, namedtuple
from dataclasses import dataclass, field

N_ROBOTS = 5
MEDICAMENTOS = ('AA', 'AB', 'BB')

# Estados de un robot (contrato del documento)
ESPERANDO, EJECUTANDO, FINALIZO, ERROR = 0, 1, 2, 3
DESCONOCIDO = -1  # interno: hay que esperar un nuevo estado 0 del robot

# Estados de la planta
OPERANDO = 'OPERANDO'
FALLO = 'FALLO'
PARADA_GLOBAL = 'PARADA_GLOBAL'

# Lo que el nucleo le pide al nodo que publique
Salida = namedtuple('Salida', ['tipo', 'datos'])  # tipo: cmd, progreso, alerta, log


@dataclass
class Parametros:
    """Tiempos y reintentos. Se cambian desde config/brain_params.yaml."""
    t_ack: float = 2.0                # espera por el estado 1 tras cada intento
    max_intentos: int = 3             # intentos de una orden antes de declarar fallo
    t_tarea: tuple = (30.0,) * N_ROBOTS  # tiempo maximo de la etapa de cada robot
    t_offline: float = 3.0            # sin estado por mas de esto = desconectado


@dataclass
class Caja:
    id: int
    medicamento: str
    ok: int = 0                       # etapas completadas
    saltadas: list = field(default_factory=list)


@dataclass
class Orden:
    caja: Caja
    seq: int
    tarea: str
    medicamento: str
    intentos: int = 1
    t_envio: float = 0.0              # tiempo operativo del ultimo envio
    t_inicio: float = 0.0             # tiempo operativo en que el robot acepto
    fallida: bool = False


class Robot:
    def __init__(self, robot_id, now):
        self.id = robot_id
        self.estado = DESCONOCIDO
        self.online = False
        self.offline_alertado = False
        self.last_seen = now
        self.cola = deque()           # cajas esperando a este robot
        self.pending = None           # orden enviada, sin confirmar
        self.activa = None            # orden confirmada (estado 1), sin terminar
        self.error_reportado = False

    def orden(self):
        return self.pending or self.activa


class NucleoBrain:
    def __init__(self, params, now):
        if len(params.t_tarea) != N_ROBOTS:
            raise ValueError(f't_tarea debe tener {N_ROBOTS} valores')
        self.p = params
        self.robots = [Robot(i, now) for i in range(1, N_ROBOTS + 1)]
        self.estado = OPERANDO
        self.estop_activo = False
        self.seq = 0
        self.ids_usados = set()
        self._salidas = []
        # El tiempo de pausa no cuenta para T_ack ni T_tarea ("tiempo operativo")
        self._t_pausa = None
        self._t_pausado_total = 0.0

    # ------------------------------------------------------------------ utilidades
    def retirar_salidas(self):
        s, self._salidas = self._salidas, []
        return s

    def _log(self, texto):
        self._salidas.append(Salida('log', texto))

    def _alerta(self, robot_id, caja_id, causa):
        self._log(f'ALERTA robot={robot_id} caja={caja_id}: {causa}')
        self._salidas.append(Salida('alerta', dict(robot_id=robot_id, caja_id=caja_id, causa=causa)))

    def _cmd(self, robot_id, caja_id, tarea, medicamento='', seq=None):
        if seq is None:
            self.seq += 1
            seq = self.seq
        self._salidas.append(Salida('cmd', dict(robot_id=robot_id, caja_id=caja_id, tarea=tarea,
                                                medicamento=medicamento, seq=seq)))
        return seq

    def _t_op(self, now):
        en_pausa = (now - self._t_pausa) if self._t_pausa is not None else 0.0
        return now - self._t_pausado_total - en_pausa

    # ------------------------------------------------------------------ GUI
    def iniciar_caja(self, caja_id, medicamento, now):
        medicamento = (medicamento or '').strip().upper()
        if self.estado != OPERANDO:
            return False, f'La planta esta en {self.estado}; no se aceptan cajas'
        if caja_id <= 0:
            return False, 'caja_id debe ser mayor que 0'
        if caja_id in self.ids_usados:
            return False, f'La caja {caja_id} ya fue iniciada'
        if medicamento not in MEDICAMENTOS:
            return False, 'Medicamento invalido: debe ser AA, AB o BB'
        self.ids_usados.add(caja_id)
        self.robots[0].cola.append(Caja(caja_id, medicamento))
        self._log(f'Caja {caja_id} aceptada (medicamento {medicamento})')
        return True, ''

    def on_estop(self, activo, now):
        if activo:
            if not self.estop_activo:
                self.estop_activo = True
                self._log('Parada global ACTIVADA')
            self._detener(PARADA_GLOBAL, now)
        elif self.estop_activo:
            self.estop_activo = False
            self._log('Parada global liberada; falta que la GUI mande continuar')

    def continuar(self, now):
        if self.estado == OPERANDO:
            return False, 'La planta ya esta operando'
        if self.estop_activo:
            return False, 'La parada global sigue activa; liberela (false) antes de continuar'
        for r in self.robots:
            orden_fallida = next((o for o in (r.pending, r.activa) if o and o.fallida), None)
            if orden_fallida or r.estado == ERROR:
                # Se salta la etapa fallida y el robot recibe ABORTAR (posicion segura)
                r.pending = r.activa = None
                if orden_fallida:
                    self._avanzar(orden_fallida.caja, r.id, completada=False)
                self._cmd(r.id, 0, 'ABORTAR')
                r.estado = DESCONOCIDO  # hasta que publique un nuevo estado 0
            else:
                self._cmd(r.id, 0, 'REANUDAR')
        self._reanudar(now)
        self._log('Planta reanudada')
        return True, 'Planta reanudada'

    # ------------------------------------------------------------------ robots
    def on_estado(self, robot_id, estado, caja_id, seq, now):
        if not 1 <= robot_id <= N_ROBOTS or estado not in (ESPERANDO, EJECUTANDO, FINALIZO, ERROR):
            return
        r = self.robots[robot_id - 1]
        r.last_seen = now
        r.online = True
        r.offline_alertado = False
        r.estado = estado

        if estado == ERROR:
            if not r.error_reportado:  # el robot repite el 3 cada segundo: una sola alerta
                r.error_reportado = True
                self._fallar(r, f'El robot {r.id} reporto un error', caja_id, now)
            return
        r.error_reportado = False

        if estado == EJECUTANDO:
            p = r.pending
            if p and seq == p.seq and caja_id == p.caja.id:
                self._aceptar(r, now)
        elif estado == FINALIZO:
            p = r.pending
            if p and seq == p.seq and caja_id == p.caja.id:  # termino sin que viéramos el 1
                self._aceptar(r, now)
            a = r.activa
            if a and seq == a.seq and caja_id == a.caja.id:
                r.activa = None
                self._avanzar(a.caja, r.id, completada=True)
        # estado 0: solo actualiza la disponibilidad. Si llega con una orden activa
        # (el robot no deberia hacerlo) se ignora y T_tarea cubre el caso.

    def _aceptar(self, r, now):
        o = r.pending
        r.pending = None
        o.fallida = False  # confirmo tarde: ya no se considera fallo de confirmacion
        o.t_inicio = self._t_op(now)
        r.activa = o

    def _avanzar(self, caja, etapa, completada):
        """La caja pasa al siguiente robot; la etapa cuenta o se salta."""
        if completada:
            caja.ok += 1
            self._salidas.append(Salida('progreso', dict(caja_id=caja.id, etapa=etapa,
                                                         porcentaje=20 * caja.ok)))
            self._log(f'Caja {caja.id}: etapa {etapa} completada ({20 * caja.ok} %)')
        else:
            caja.saltadas.append(etapa)
            self._log(f'Caja {caja.id}: etapa {etapa} saltada')
        if etapa < N_ROBOTS:
            self.robots[etapa].cola.append(caja)  # robots[etapa] es el robot etapa+1
        elif caja.saltadas:
            etapas = ', '.join(str(e) for e in caja.saltadas)
            self._alerta(0, caja.id, f'La caja {caja.id} termino incompleta (etapas saltadas: {etapas})')
        else:
            self._log(f'Caja {caja.id} terminada')

    # ------------------------------------------------------------------ fallos
    def _fallar(self, r, causa, caja_id, now):
        o = r.orden()
        if o:
            o.fallida = True
            caja_id = o.caja.id
        self._alerta(r.id, caja_id, causa)
        self._detener(FALLO, now)

    def _detener(self, nuevo, now):
        if self.estado == OPERANDO:
            self._t_pausa = now
            self.estado = nuevo
            for r in self.robots:
                self._cmd(r.id, 0, 'PAUSAR')
        elif nuevo == PARADA_GLOBAL:
            self.estado = PARADA_GLOBAL

    def _reanudar(self, now):
        if self._t_pausa is not None:
            self._t_pausado_total += now - self._t_pausa
            self._t_pausa = None
        self.estado = OPERANDO

    # ------------------------------------------------------------------ ciclo periodico
    def tick(self, now):
        for r in self.robots:
            if not r.offline_alertado and now - r.last_seen > self.p.t_offline:
                self._offline(r, now)

        if self.estado == OPERANDO:
            op = self._t_op(now)
            for r in self.robots:
                p, a = r.pending, r.activa
                if p and not p.fallida and op - p.t_envio >= self.p.t_ack:
                    if p.intentos < self.p.max_intentos:
                        p.intentos += 1
                        p.t_envio = op
                        self._cmd(r.id, p.caja.id, p.tarea, p.medicamento, seq=p.seq)
                        self._log(f'Robot {r.id}: reintento {p.intentos} de la caja {p.caja.id}')
                    else:
                        self._fallar(r, f'El robot {r.id} no confirmo la orden tras '
                                        f'{self.p.max_intentos} intentos', p.caja.id, now)
                elif a and not a.fallida and op - a.t_inicio >= self.p.t_tarea[r.id - 1]:
                    self._fallar(r, f'El robot {r.id} no termino la etapa en '
                                    f'{self.p.t_tarea[r.id - 1]:g} s', a.caja.id, now)
                if self.estado != OPERANDO:
                    break
        self._despachar(now)

    def _offline(self, r, now):
        r.online = False
        r.offline_alertado = True
        tiene_trabajo = bool(r.pending or r.activa or r.cola)
        causa = f'El robot {r.id} perdio la comunicacion (sin estado por mas de {self.p.t_offline:g} s)'
        o = r.orden()
        if o:
            o.fallida = True
        if tiene_trabajo:
            self._fallar(r, causa, 0, now)
        else:
            self._alerta(r.id, 0, causa)  # estaba libre: solo se avisa

    def _despachar(self, now):
        """La caja k pasa al robot i cuando el i-1 la finalizo y el i esta esperando."""
        if self.estado != OPERANDO:
            return
        for r in self.robots:
            if r.online and r.estado == ESPERANDO and not r.pending and not r.activa and r.cola:
                caja = r.cola.popleft()
                medicamento = caja.medicamento if r.id == 2 else ''
                tarea = f'T{r.id}'
                seq = self._cmd(r.id, caja.id, tarea, medicamento)
                r.pending = Orden(caja, seq, tarea, medicamento, t_envio=self._t_op(now))
                self._log(f'Caja {caja.id} -> robot {r.id} (seq {seq})')

    # ------------------------------------------------------------------ estado para la GUI
    def snapshot(self):
        estados, cajas, colas = [], [], []
        for r in self.robots:
            if not r.online:
                estados.append(ERROR)  # sin comunicacion se muestra como error
            else:
                estados.append(r.estado if r.estado in (ESPERANDO, EJECUTANDO, FINALIZO, ERROR) else ESPERANDO)
            o = r.orden()
            cajas.append(o.caja.id if o else 0)
            colas.append(len(r.cola))
        return dict(estado=self.estado, estados_robots=estados, cajas_robots=cajas, cajas_en_cola=colas)
