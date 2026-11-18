"""Pruebas del nucleo del brain con 5 robots falsos y un reloj simulado.

No necesitan ROS:   cd src/brain && python3 -m pytest test -v
"""
from brain.brain_core import (ERROR, FALLO, OPERANDO, PARADA_GLOBAL, NucleoBrain, Parametros)
from brain.robot_falso import RobotFalso


class Sim:
    """Une el nucleo con los robots falsos. Los mensajes llegan en el paso siguiente."""

    def __init__(self, duracion=2.0, params=None):
        self.t = 0.0
        self.nucleo = NucleoBrain(params or Parametros(), self.t)
        self.robots = {i: RobotFalso(i, duracion=duracion) for i in range(1, 6)}
        self.silenciados = set()      # robots que dejan de publicar estado (desconexion)
        self.cmds, self.progreso, self.alertas = [], [], []
        self._sig_pub = 0.0
        self._publicar(forzar=True)

    def _publicar(self, forzar=False):
        periodico = self.t >= self._sig_pub
        if periodico:
            self._sig_pub += 1.0
        for i, r in self.robots.items():
            if (periodico or r.publicar or forzar) and i not in self.silenciados:
                self.nucleo.on_estado(*r.mensaje(), self.t)
            r.publicar = False

    def _rutear(self):
        for s in self.nucleo.retirar_salidas():
            d = s.datos
            if s.tipo == 'cmd':
                self.cmds.append(d)
                self.robots[d['robot_id']].on_cmd(d['tarea'], d['caja_id'], d['seq'], d['medicamento'])
            elif s.tipo == 'progreso':
                self.progreso.append(d)
            elif s.tipo == 'alerta':
                self.alertas.append(d)

    def correr(self, segundos, dt=0.1):
        for _ in range(round(segundos / dt)):
            self.t += dt
            for r in self.robots.values():
                r.tick(dt, self.t)
            self._publicar()
            self.nucleo.tick(self.t)
            self._rutear()

    def iniciar(self, caja_id, med='AB'):
        r = self.nucleo.iniciar_caja(caja_id, med, self.t)
        self._rutear()
        return r

    def continuar(self):
        r = self.nucleo.continuar(self.t)
        self._rutear()
        return r

    def estop(self, activo):
        self.nucleo.on_estop(activo, self.t)
        for r in self.robots.values():
            r.on_estop(activo)
        self._rutear()

    def tareas(self, tarea):
        return [c for c in self.cmds if c['tarea'] == tarea]


def porcentajes(sim, caja):
    return [p['porcentaje'] for p in sim.progreso if p['caja_id'] == caja]


# ---------------------------------------------------------------- flujo normal
def test_caja_completa_pasa_por_5_etapas_en_orden():
    s = Sim()
    s.correr(1.5)  # los robots se presentan
    assert s.iniciar(17) == (True, '')
    s.correr(25)
    assert porcentajes(s, 17) == [20, 40, 60, 80, 100]
    assert [p['etapa'] for p in s.progreso] == [1, 2, 3, 4, 5]
    assert s.alertas == []
    assert s.nucleo.estado == OPERANDO


def test_cada_robot_recibe_su_tarea_y_solo_el_2_el_medicamento():
    s = Sim()
    s.correr(1.5)
    s.iniciar(17, 'ab')  # tambien acepta minusculas
    s.correr(25)
    ordenes = [c for c in s.cmds if c['tarea'].startswith('T')]
    assert [c['tarea'] for c in ordenes] == ['T1', 'T2', 'T3', 'T4', 'T5']
    assert [c['medicamento'] for c in ordenes] == ['', 'AB', '', '', '']


def test_varias_cajas_en_la_linea_a_la_vez():
    s = Sim(duracion=3.0)
    s.correr(1.5)
    for caja in (1, 2, 3):
        assert s.iniciar(caja)[0]
    s.correr(8)
    ocupados = [r for r in s.nucleo.robots if r.activa or r.pending]
    assert len(ocupados) >= 2   # hay cajas en etapas distintas al mismo tiempo
    s.correr(40)
    for caja in (1, 2, 3):
        assert porcentajes(s, caja) == [20, 40, 60, 80, 100]


def test_las_etapas_respetan_el_orden_por_caja():
    s = Sim(duracion=2.0)
    s.correr(1.5)
    s.iniciar(1)
    s.iniciar(2)
    s.correr(40)
    for caja in (1, 2):
        etapas = [p['etapa'] for p in s.progreso if p['caja_id'] == caja]
        assert etapas == [1, 2, 3, 4, 5]


# ---------------------------------------------------------------- iniciar_caja
def test_iniciar_caja_rechaza_datos_invalidos():
    s = Sim()
    s.correr(1.5)
    assert s.iniciar(0)[0] is False
    assert s.iniciar(5, 'XX')[0] is False
    assert s.iniciar(5, '')[0] is False
    assert s.iniciar(5, 'AA')[0] is True
    ok, motivo = s.iniciar(5, 'AA')
    assert ok is False and 'ya fue iniciada' in motivo


def test_iniciar_caja_rechazada_si_la_planta_no_opera():
    s = Sim()
    s.correr(1.5)
    s.estop(True)
    assert s.iniciar(9) == (False, 'La planta esta en PARADA_GLOBAL; no se aceptan cajas')


# ---------------------------------------------------------------- fallos
def test_robot_que_no_confirma_3_intentos_alerta_y_para_todo():
    s = Sim()
    s.correr(1.5)
    s.robots[3].mudo = True
    s.iniciar(1)
    s.correr(15)
    t3 = s.tareas('T3')
    assert len(t3) == 3 and len({c['seq'] for c in t3}) == 1   # mismo seq en los 3 intentos
    assert [(a['robot_id'], a['caja_id']) for a in s.alertas] == [(3, 1)]
    assert 'no confirmo' in s.alertas[0]['causa']
    assert s.nucleo.estado == FALLO
    assert {c['robot_id'] for c in s.tareas('PAUSAR')} == {1, 2, 3, 4, 5}
    assert s.iniciar(2) == (False, 'La planta esta en FALLO; no se aceptan cajas')


def test_continuar_salta_la_etapa_fallida_y_la_caja_nunca_llega_a_100():
    s = Sim()
    s.correr(1.5)
    s.robots[3].mudo = True
    s.iniciar(1)
    s.correr(15)
    s.robots[3].mudo = False   # el grupo del robot 3 lo arreglo
    assert s.continuar() == (True, 'Planta reanudada')
    assert [c['robot_id'] for c in s.tareas('ABORTAR')] == [3]
    assert {c['robot_id'] for c in s.tareas('REANUDAR')} == {1, 2, 4, 5}
    s.correr(25)
    assert porcentajes(s, 1) == [20, 40, 60, 80]          # sin la etapa 3, nunca 100
    assert (0, 1) in [(a['robot_id'], a['caja_id']) for a in s.alertas]
    assert 'incompleta' in s.alertas[-1]['causa']
    assert s.nucleo.estado == OPERANDO


def test_robot_con_error_una_sola_alerta_aunque_repita_el_estado_3():
    s = Sim()
    s.correr(1.5)
    s.robots[4].error_en_caja = 1
    s.iniciar(1)
    s.correr(20)
    assert [(a['robot_id'], a['caja_id']) for a in s.alertas] == [(4, 1)]
    assert s.nucleo.estado == FALLO
    assert s.nucleo.snapshot()['estados_robots'][3] == ERROR


def test_etapa_que_excede_t_tarea_alerta_y_para():
    s = Sim(params=Parametros(t_tarea=(30.0, 5.0, 30.0, 30.0, 30.0)))
    s.correr(1.5)
    s.robots[2].duracion = 50.0
    s.iniciar(1)
    s.correr(20)
    assert [(a['robot_id'], a['caja_id']) for a in s.alertas] == [(2, 1)]
    assert 'no termino la etapa' in s.alertas[0]['causa']
    assert s.nucleo.estado == FALLO


def test_robot_libre_que_se_desconecta_solo_alerta():
    s = Sim()
    s.correr(2)
    s.silenciados.add(5)
    s.correr(5)
    assert [(a['robot_id'], a['caja_id']) for a in s.alertas] == [(5, 0)]
    assert s.nucleo.estado == OPERANDO
    assert s.nucleo.snapshot()['estados_robots'][4] == ERROR   # se muestra como error


def test_robot_con_trabajo_que_se_desconecta_para_la_planta():
    s = Sim(duracion=5.0)
    s.correr(1.5)
    s.iniciar(1)
    s.correr(2)
    s.silenciados.add(1)
    s.correr(5)
    assert s.alertas and s.alertas[0]['robot_id'] == 1
    assert s.nucleo.estado == FALLO


# ---------------------------------------------------------------- parada global
def test_estop_detiene_y_no_se_reanuda_sin_liberar_y_continuar():
    s = Sim(duracion=3.0)
    s.correr(1.5)
    s.iniciar(1)
    s.correr(2)
    s.estop(True)
    assert s.nucleo.estado == PARADA_GLOBAL
    avance = len(s.progreso)
    s.correr(10)
    assert len(s.progreso) == avance                       # nada avanza
    assert s.continuar() == (False, 'La parada global sigue activa; liberela (false) antes de continuar')
    s.estop(False)
    assert s.nucleo.estado == PARADA_GLOBAL                # liberar no basta
    assert s.continuar()[0] is True
    s.correr(25)
    assert porcentajes(s, 1) == [20, 40, 60, 80, 100]      # retoma donde estaba


def test_el_tiempo_en_pausa_no_cuenta_para_t_tarea():
    s = Sim(duracion=4.0, params=Parametros(t_tarea=(6.0,) * 5))
    s.correr(1.5)
    s.iniciar(1)
    s.correr(2)
    s.estop(True)
    s.correr(60)                                           # una parada larguisima
    s.estop(False)
    s.continuar()
    s.correr(30)
    assert [a for a in s.alertas if 'no termino' in a['causa']] == []
    assert porcentajes(s, 1) == [20, 40, 60, 80, 100]


# ---------------------------------------------------------------- mensajes viejos y duplicados
def test_seq_viejo_o_duplicado_se_ignora():
    s = Sim(duracion=3.0)
    s.correr(1.5)
    s.iniciar(1)
    s.correr(1)
    n = s.nucleo.robots[0]
    assert n.activa is not None
    seq = n.activa.seq
    s.nucleo.on_estado(1, 2, 1, seq + 99, s.t)             # finalizo con otro seq: no vale
    s.nucleo.on_estado(1, 2, 999, seq, s.t)                # otra caja: no vale
    assert s.progreso == []
    s.correr(30)
    assert porcentajes(s, 1) == [20, 40, 60, 80, 100]      # los duplicados del 2 no suman de mas


def test_snapshot_tiene_el_formato_del_documento():
    s = Sim(duracion=5.0)
    s.correr(1.5)
    s.iniciar(18)
    s.iniciar(19)
    s.correr(1)
    snap = s.nucleo.snapshot()
    assert snap['estado'] == OPERANDO
    assert len(snap['estados_robots']) == len(snap['cajas_robots']) == len(snap['cajas_en_cola']) == 5
    assert snap['cajas_robots'][0] == 18 and snap['cajas_en_cola'][0] == 1
