"""Pruebas de la lógica del Robot 1 con un trajectory_publisher simulado (sin ROS).

    cd robot_control && python3 -m pytest test -v
"""
from robot_control.robot1_core import ERROR, ESPERANDO, EJECUTANDO, FINALIZO, Parametros, Robot1Core


class TrajSim:
    """Réplica de la máquina de estados de trajectory_publisher (con ~/reset)."""

    def __init__(self, T=16.9, t_retorno=6.0):
        self.estado, self.t, self.T, self.t_retorno, self._ret = 'espera', 0.0, T, t_retorno, 0.0
        self.sordo = False        # ignora ~/start (no arranca)
        self.falla_start = None   # mensaje de error para ~/start

    def start(self):
        if self.falla_start:
            return False, self.falla_start
        if self.sordo:
            return True, 'ok'
        if self.estado == 'volviendo':
            return False, 'Está volviendo al inicio'
        if self.estado == 'corriendo':
            return False, 'Ya está corriendo'
        if self.estado == 'fin':
            self.t = 0.0
        self.estado = 'corriendo'
        return True, 'ok'

    def stop(self):
        if self.estado != 'corriendo':
            return False, 'No está corriendo'
        self.estado = 'pausa'
        return True, 'ok'

    def reset(self):
        if self.estado in ('corriendo', 'volviendo'):
            return False, 'no'
        if self.estado == 'espera':
            return True, 'ok'
        self._ret = 0.0 if self.estado == 'fin' else self.t_retorno
        self.estado = 'volviendo'
        return True, 'ok'

    def tick(self, dt):
        if self.estado == 'corriendo':
            self.t += dt
            if self.t >= self.T:
                self.t, self.estado = self.T, 'fin'
        elif self.estado == 'volviendo':
            self._ret -= dt
            if self._ret <= 0:
                self.estado, self.t = 'espera', 0.0


class Sim:
    def __init__(self, params=None, **traj):
        self.t = 0.0
        self.core = Robot1Core(params or Parametros(), self.t)
        self.traj = TrajSim(**traj)
        self.err, self.link, self.latidos = 0.005, True, False
        self.pubs, self.srvs, self.logs = [], [], []
        self.paso(0.05)

    def _volcar(self):
        for s in self.core.retirar_salidas():
            if s.tipo == 'publicar':
                self.pubs.append(self.core.mensaje())
            elif s.tipo == 'srv':
                self.srvs.append(s.dato)
                ok, msg = getattr(self.traj, s.dato)()
                self.core.on_srv_resultado(s.dato, ok, msg, self.t)
            else:
                self.logs.append(s.dato)

    def paso(self, dt=0.05):
        self.t += dt
        self.traj.tick(dt)
        self.core.on_traj_estado(self.traj.estado, self.t)
        self.core.on_tracking_error(self.err, self.t)
        if self.link:
            self.core.on_joint_states(self.t)
        if self.latidos:
            self.core.on_latido(self.t)
        self.core.tick(self.t)
        self._volcar()

    def correr(self, s):
        for _ in range(round(s / 0.05)):
            self.paso()

    def cmd(self, tarea, caja=0, seq=0):
        self.core.on_cmd(tarea, caja, seq, self.t)
        self._volcar()

    def estop(self, activo):
        self.core.on_estop(activo, self.t)
        self._volcar()

    def estados(self):
        """Secuencia de estados publicados, sin repetidos consecutivos."""
        out = []
        for _, e, _, _ in self.pubs:
            if not out or out[-1] != e:
                out.append(e)
        return out


def test_no_publica_estado_hasta_ver_la_esp32_y_trajectory_publisher():
    core = Robot1Core(Parametros(), 0.0)
    assert core.mensaje() is None
    core.on_joint_states(0.1)
    assert core.mensaje() is None
    core.on_traj_estado('espera', 0.1)
    assert core.mensaje() == (1, ESPERANDO, 0, 0)


def test_ciclo_normal_1_2_0_y_arranca_una_sola_vez():
    s = Sim()
    s.cmd('T1', 17, 41)
    assert s.pubs[0] == (1, EJECUTANDO, 17, 41)      # confirmacion inmediata
    s.correr(25)
    assert s.estados() == [EJECUTANDO, FINALIZO, ESPERANDO]
    assert s.pubs[-1] == (1, ESPERANDO, 0, 41)       # libre: caja 0, mismo seq
    assert s.srvs == ['start']


def test_reintento_con_el_mismo_seq_no_repite_la_tarea():
    s = Sim()
    s.cmd('T1', 17, 41)
    s.correr(3)
    s.cmd('T1', 17, 41)
    assert s.srvs == ['start'] and s.pubs[-1] == (1, EJECUTANDO, 17, 41)


def test_duplicado_o_viejo_se_ignora_aun_despues_de_terminar():
    s = Sim()
    s.cmd('T1', 17, 41)
    s.correr(25)
    n = len(s.srvs)
    s.cmd('T1', 17, 41)
    s.cmd('T1', 16, 40)
    s.correr(2)
    assert len(s.srvs) == n and s.core.estado == ESPERANDO


def test_ordenes_de_otros_robots_se_ignoran():
    s = Sim()
    s.cmd('T3', 17, 5)
    assert s.srvs == [] and s.core.estado == ESPERANDO


def test_pausar_detiene_y_reanudar_retoma_donde_estaba():
    s = Sim()
    s.cmd('T1', 17, 41)
    s.correr(5)
    s.cmd('PAUSAR')
    s.correr(1)
    t_pausa = s.traj.t
    s.correr(20)
    assert s.traj.estado == 'pausa' and s.traj.t == t_pausa and s.core.estado == EJECUTANDO
    s.cmd('REANUDAR')
    s.correr(20)
    assert s.estados() == [EJECUTANDO, FINALIZO, ESPERANDO]
    assert s.srvs.count('start') == 2          # inicio + reanudacion


def test_parada_global_para_y_no_reanuda_sin_reanudar():
    s = Sim()
    s.cmd('T1', 17, 41)
    s.correr(5)
    s.estop(True)
    s.correr(1)
    assert s.traj.estado == 'pausa'
    s.estop(False)
    s.correr(5)
    assert s.traj.estado == 'pausa'            # liberar no reanuda
    s.cmd('REANUDAR')
    s.correr(20)
    assert s.core.estado == ESPERANDO and s.pubs[-1][1] == ESPERANDO


def test_reanudar_con_la_parada_activa_se_ignora():
    s = Sim()
    s.cmd('T1', 17, 41)
    s.correr(5)
    s.estop(True)
    s.cmd('REANUDAR')
    s.correr(3)
    assert s.traj.estado == 'pausa'


def test_t1_durante_parada_global_se_ignora():
    s = Sim()
    s.estop(True)
    s.cmd('T1', 17, 41)
    assert s.srvs == [] and s.core.estado == ESPERANDO


def test_sin_latido_del_brain_parada_segura():
    s = Sim()
    s.latidos = True
    s.correr(1)
    s.cmd('T1', 17, 41)
    s.correr(4)
    s.latidos = False
    s.correr(5)                                 # pasan mas de 3 s sin latido
    assert s.traj.estado == 'pausa'
    s.latidos = True
    s.correr(5)
    assert s.traj.estado == 'pausa'            # con tarea, espera REANUDAR
    s.cmd('REANUDAR')
    s.correr(20)
    assert s.core.estado == ESPERANDO


def test_error_de_seguimiento_reporta_3_una_vez_y_mantiene_posicion():
    s = Sim()
    s.cmd('T1', 17, 41)
    s.correr(5)
    s.err = 0.2
    s.correr(2)
    assert s.core.estado == ERROR and s.traj.estado == 'pausa'
    assert s.estados() == [EJECUTANDO, ERROR]


def test_abortar_tras_error_vuelve_a_home_y_publica_0():
    s = Sim()
    s.cmd('T1', 17, 41)
    s.correr(5)
    s.err = 0.2
    s.correr(1)
    s.err = 0.005                               # el operador dejo el robot en home
    s.cmd('ABORTAR')
    s.correr(10)
    assert s.srvs[-1] == 'reset' and s.traj.estado == 'espera'
    assert s.pubs[-1] == (1, ESPERANDO, 0, 41) and s.core.pausado is False


def test_abortar_en_plena_tarea_detiene_y_regresa():
    s = Sim()
    s.cmd('T1', 17, 41)
    s.correr(6)
    s.cmd('ABORTAR')
    s.correr(12)
    assert s.srvs == ['start', 'stop', 'reset']
    assert s.traj.estado == 'espera' and s.core.estado == ESPERANDO


def test_abortar_con_el_robot_libre_deja_todo_igual():
    s = Sim()
    s.cmd('ABORTAR')
    s.correr(2)
    assert s.core.estado == ESPERANDO and s.traj.estado == 'espera'


def test_abortar_sin_volver_a_home_termina_en_error():
    s = Sim(params=Parametros(t_asentar_s=5.0))
    s.cmd('T1', 17, 41)
    s.correr(6)
    s.err = 0.2                                 # el robot no regresa
    s.cmd('ABORTAR')
    s.correr(14)
    assert s.core.estado == ERROR


def test_enlace_con_la_esp32_caido_es_error():
    s = Sim()
    s.correr(1)
    s.link = False
    s.correr(2)
    assert s.core.estado == ERROR


def test_t1_con_el_robot_fuera_de_home_da_error_y_no_arranca():
    s = Sim()
    s.err = 0.1
    s.paso()                                   # el nucleo se entera del error
    s.cmd('T1', 17, 41)
    assert s.core.estado == ERROR and s.srvs == []
    assert s.pubs[-1] == (1, ERROR, 17, 41)


def test_si_start_falla_es_error():
    s = Sim()
    s.traj.falla_start = 'servicio no disponible'
    s.cmd('T1', 17, 41)
    s.correr(1)
    assert s.core.estado == ERROR


def test_si_la_trayectoria_no_arranca_es_error():
    s = Sim()
    s.traj.sordo = True
    s.cmd('T1', 17, 41)
    s.correr(3)
    assert s.core.estado == ERROR


def test_no_pasa_a_esperando_si_no_vuelve_a_home():
    s = Sim(params=Parametros(t_asentar_s=3.0))
    s.cmd('T1', 17, 41)
    s.correr(10)
    s.err = 0.05                                # converge mal: > e_inicio_m
    s.correr(12)
    assert s.core.estado == ERROR


def test_t1_estando_pausado_confirma_y_arranca_al_reanudar():
    s = Sim()
    s.cmd('PAUSAR')
    s.cmd('T1', 17, 41)
    s.correr(2)
    assert s.pubs[-1][1] == EJECUTANDO and s.srvs == []
    s.cmd('REANUDAR')
    s.correr(25)
    assert s.srvs[0] == 'start' and s.core.estado == ESPERANDO
