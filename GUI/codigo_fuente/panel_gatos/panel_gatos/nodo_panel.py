#!/usr/bin/env python3

import json
import os
import random
import sqlite3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, Float32, Int32, String

DB_PATH = os.path.expanduser('~/panel.db')
P = '/panel_control'
ROBOTS = ['robot1', 'robot2', 'robot3', 'robot4', 'robot5']
VARIABLES_INICIALES = [('temperatura', 'C', 25.0), ('humedad', '%', 55.0), ('presion', 'kPa', 101.0),
                       ('velocidad', 'm/s', 0.5), ('corriente', 'A', 2.0), ('voltaje', 'V', 12.0)]
AVISOS = [('info', 'Ciclo de dosificación completado.'),
          ('advertencia', 'Nivel bajo en el contenedor de medicamento AB.'),
          ('error', 'Sensor de la banda transportadora sin respuesta.')]


class NodoPanel(Node):
    def __init__(self):
        super().__init__('nodo_panel')
        self.db = sqlite3.connect(DB_PATH)
        self.crear_bd()

        self.estado = 'detenido'
        self.entran = 0
        self.salen = 0
        self.progreso = {r: 0.0 for r in ROBOTS}
        self.vars = {}  # nombre -> {'pub': publisher, 'valor': float}

        # Publicadores
        self.pub_estado = self.create_publisher(String, f'{P}/estado', 10)
        self.pub_entran = self.create_publisher(Int32, f'{P}/cajas/entran', 10)
        self.pub_salen = self.create_publisher(Int32, f'{P}/cajas/salen', 10)
        self.pub_avisos = self.create_publisher(String, f'{P}/avisos', 10)
        self.pub_robot = {r: self.create_publisher(Float32, f'{P}/robot/{r}/progreso', 10) for r in ROBOTS}

        # Comandos que manda el panel
        self.create_subscription(Empty, f'{P}/cmd/iniciar', lambda m: self.cambiar_estado('ejecutando'), 10)
        self.create_subscription(Empty, f'{P}/cmd/detener', lambda m: self.cambiar_estado('detenido'), 10)
        self.create_subscription(Empty, f'{P}/cmd/emergencia', lambda m: self.cambiar_estado('emergencia'), 10)
        self.create_subscription(String, f'{P}/orden_medicamento', self.on_medicamento, 10)

        # Temporizadores
        self.create_timer(3.0, self.sincronizar_variables)  # detecta variables nuevas en la BD
        self.create_timer(1.0, self.tick_variables)
        self.create_timer(1.0, self.tick_robots)
        self.create_timer(4.0, self.tick_cajas)
        self.create_timer(15.0, self.tick_avisos)

        self.sincronizar_variables()
        self.get_logger().info(f'Listo. BD: {DB_PATH}. Variables: {list(self.vars)}')

    # ---------- Base de datos ----------
    def crear_bd(self):
        c = self.db
        c.executescript("""
            CREATE TABLE IF NOT EXISTS variables(nombre TEXT PRIMARY KEY, unidad TEXT, base REAL);
            CREATE TABLE IF NOT EXISTS lecturas(id INTEGER PRIMARY KEY AUTOINCREMENT, nombre TEXT,
                                                valor REAL, ts TEXT DEFAULT CURRENT_TIMESTAMP);
            CREATE TABLE IF NOT EXISTS cajas(id INTEGER PRIMARY KEY AUTOINCREMENT, entran INTEGER,
                                             salen INTEGER, ts TEXT DEFAULT CURRENT_TIMESTAMP);
            CREATE TABLE IF NOT EXISTS comandos(id INTEGER PRIMARY KEY AUTOINCREMENT, comando TEXT,
                                                ts TEXT DEFAULT CURRENT_TIMESTAMP);
        """)
        if c.execute('SELECT COUNT(*) FROM variables').fetchone()[0] == 0:
            c.executemany('INSERT INTO variables VALUES (?,?,?)', VARIABLES_INICIALES)
        c.commit()

    def sincronizar_variables(self):
        """Crea un publicador por cada variable de la BD (y borra los de variables eliminadas)."""
        en_bd = {n: b for n, b in self.db.execute('SELECT nombre, base FROM variables')}
        for nombre, base in en_bd.items():
            if nombre not in self.vars:
                pub = self.create_publisher(Float32, f'{P}/variable/{nombre}', 10)
                self.vars[nombre] = {'pub': pub, 'valor': float(base or 0.0)}
                self.get_logger().info(f'Variable publicada: {nombre}')
        for nombre in list(self.vars):
            if nombre not in en_bd:
                self.destroy_publisher(self.vars.pop(nombre)['pub'])
                self.get_logger().info(f'Variable retirada: {nombre}')

    # ---------- Comandos ----------
    def cambiar_estado(self, nuevo):
        self.estado = nuevo
        self.pub_estado.publish(String(data=nuevo))
        self.log_comando(nuevo)

    def on_medicamento(self, msg):
        self.log_comando(f'medicamento:{msg.data}')

    def log_comando(self, texto):
        self.db.execute('INSERT INTO comandos(comando) VALUES (?)', (texto,))
        self.db.commit()
        self.get_logger().info(f'Comando recibido: {texto}')

    # ---------- Simulación ----------
    def tick_variables(self):
        for nombre, v in self.vars.items():
            v['valor'] += random.uniform(-1.0, 1.0)
            self.db.execute('INSERT INTO lecturas(nombre, valor) VALUES (?,?)', (nombre, v['valor']))
            v['pub'].publish(Float32(data=float(v['valor'])))
        self.db.commit()

    def tick_robots(self):
        if self.estado != 'ejecutando':
            return
        for r in ROBOTS:
            self.progreso[r] = 0.0 if self.progreso[r] >= 100 else min(100.0, self.progreso[r] + random.uniform(0, 5))
            self.pub_robot[r].publish(Float32(data=self.progreso[r]))

    def tick_cajas(self):
        if self.estado != 'ejecutando':
            return
        if random.random() > 0.45:
            self.entran += 1
            self.pub_entran.publish(Int32(data=self.entran))
        elif self.salen < self.entran:
            self.salen += 1
            self.pub_salen.publish(Int32(data=self.salen))
        else:
            return
        self.db.execute('INSERT INTO cajas(entran, salen) VALUES (?,?)', (self.entran, self.salen))
        self.db.commit()

    def tick_avisos(self):
        if self.estado == 'ejecutando':
            nivel, mensaje = random.choice(AVISOS)
            self.pub_avisos.publish(String(data=json.dumps({'nivel': nivel, 'mensaje': mensaje})))


def main():
    rclpy.init()
    nodo = NodoPanel()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.db.close()
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
