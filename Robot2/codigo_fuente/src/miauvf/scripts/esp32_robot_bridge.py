#!/usr/bin/env python3
import csv
import glob
import math
import os
import threading
import time
import serial
import yaml

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

RADS_TO_RPM = 60.0 / (2.0 * math.pi)
RPM_TO_RADS = (2.0 * math.pi) / 60.0


def leer_ph_desde_csv(default_q):
    candidatos = sorted(glob.glob(os.path.expanduser("~/miau_trayectorias/PH_a_*.csv")))
    ruta = candidatos[0] if candidatos else ""
    if not ruta or not os.path.exists(ruta):
        return list(default_q)
    try:
        with open(ruta, "r", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            row = next(reader)
            return [float(row["J1_pos"]), float(row["J2_pos"]), float(row["J3_pos"])]
    except Exception:
        return list(default_q)


def buscar_calibracion_default():
    rutas = [
        os.path.expanduser("~/manipulador_3dof/ros2_ws/src/miau/config/hw_calibration.yaml"),
        os.path.expanduser("~/manipulador_3dof/ros2_ws/src/miauvf/config/hw_calibration.yaml"),
    ]
    for r in rutas:
        if os.path.exists(r):
            return r
    return rutas[0]


class ESP32RobotBridge(Node):
    def __init__(self):
        super().__init__("esp32_robot_bridge")

        puertos = glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")
        default_port = puertos[0] if puertos else "/dev/ttyUSB0"
        default_cal = buscar_calibracion_default()

        self.declare_parameter("serial_port", default_port)
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("calibration_file", default_cal)
        self.declare_parameter("cmd_timeout_s", 0.30)

        self.port = self.get_parameter("serial_port").value
        self.baud = int(self.get_parameter("baudrate").value)
        self.cal_file = self.get_parameter("calibration_file").value
        self.cmd_timeout = float(self.get_parameter("cmd_timeout_s").value)

        self.joint_names = ["J1", "J2", "J3"]
        self.cfg = self._cargar_calibracion(self.cal_file)
        self.cal_mtime = os.path.getmtime(self.cal_file) if os.path.exists(self.cal_file) else 0.0

        self.q_init = leer_ph_desde_csv([-0.489151210, 1.016052604, -2.452465863])
        self.pos_rad = list(self.q_init)
        self.vel_rads = [0.0, 0.0, 0.0]

        self.target_rpm = [0.0, 0.0, 0.0]
        self.prev_pos_rad = list(self.q_init)
        self.stall_time = [0.0, 0.0, 0.0]
        self.last_cmd_time = 0.0
        self.was_idle = False

        self.rx_j12 = False
        self.rx_j3 = False
        self.last_warn_time = 0.0

        self.lock = threading.Lock()
        self.ser_lock = threading.Lock()
        self.running = True

        self.get_logger().info(f"Conectando a ESP32 Antena en {self.port} @ {self.baud}...")
        self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
        time.sleep(2.0)
        self.ser.reset_input_buffer()

        self.pos_pub = self.create_publisher(Float64MultiArray, "/robot/posiciones", 20)
        self.js_pub = self.create_publisher(JointState, "/joint_states", 20)

        self.cmd_sub = self.create_subscription(
            Float64MultiArray, "/velocity_controller/commands", self.on_vel_cmd, 10
        )
        self.pinza_sub = self.create_subscription(
            String, "rawr", self.on_pinza_cmd, 10
        )
        self.fault_latched = False
        self.traj_start_t = 0.0
        self.prev_rpm_out = [0.0, 0.0, 0.0]
        self.en_reposo = False
        self.settle_until = 0.0
        self.holding = [False, False, False]
        self.estado_sub = self.create_subscription(
            String, "/estado_trayectoria", self.on_estado_trayectoria, 10
        )

        self.rx_thread = threading.Thread(target=self.serial_read_loop, daemon=True)
        self.rx_thread.start()

        self._subir_pid_esp32()
        self.tx_timer = self.create_timer(0.05, self.tx_loop)
        self.fb_timer = self.create_timer(0.03, self.publish_feedback)
        self.get_logger().info(f"Puente hardware listo ({self.cal_file}).")

    def _subir_pid_esp32(self):
        for idx, jname in enumerate(self.joint_names):
            c = self.cfg[idx]
            kp = float(c.get("esp32_kp", 10.0))
            ki = float(c.get("esp32_ki", 70.0))
            kd = float(c.get("esp32_kd", 0.01))
            try:
                if jname == "J2":
                    ff_up = float(c.get("ff_up", 490.0))
                    sb = float(c.get("start_boost", 70.0))
                    mdn = float(c.get("max_dn", 330.0))
                    linea = f"J2,CFG,{kp:.3f},{ki:.3f},{kd:.4f},{ff_up:.1f},{sb:.1f},{mdn:.1f}\n"
                elif jname == "J3":
                    ff_up = float(c.get("ff_up", 280.0))
                    ff_dn = float(c.get("ff_dn", 195.0))
                    linea = f"J3,CFG,{kp:.3f},{ki:.3f},{kd:.4f},{ff_up:.1f},{ff_dn:.1f}\n"
                else:
                    linea = f"J1,CFG,{kp:.3f},{ki:.3f},{kd:.4f}\n"
                self.ser.write(linea.encode("utf-8"))
                time.sleep(0.06)
            except serial.SerialException as e:
                self.get_logger().warn(f"No se pudo enviar CFG a {jname}: {e}")
        self.get_logger().info("Parámetros y compensación enviados a ESP32 (J1, J2, J3).")

    def _cargar_calibracion(self, path: str) -> list[dict]:
        defaults = [
            {
                "urdf_dir": -1.0,
                "enc_polarity": 1.0,
                "counts_per_rev": 1060.0,
                "esp32_ppr": 1060.0,
                "min_rpm_pos": 1.5,
                "min_rpm_neg": 1.5,
                "max_cmd_rpm": 20.0,
                "esp32_kp": 12.5,
                "esp32_ki": 88.0,
                "esp32_kd": 0.015,
            },
            {
                "urdf_dir": -1.0,
                "enc_polarity": 1.0,
                "counts_per_rev": 2247.0,
                "esp32_ppr": 1060.0,
                "min_rpm_pos": 2.5,
                "min_rpm_neg": 2.0,
                "max_cmd_rpm": 25.0,
                "esp32_kp": 7.5,
                "esp32_ki": 68.0,
                "esp32_kd": 0.018,
            },
            {
                "urdf_dir": -1.0,
                "enc_polarity": 1.0,
                "counts_per_rev": 2325.0,
                "esp32_ppr": 530.0,
                "min_rpm_pos": 2.0,
                "min_rpm_neg": 2.0,
                "max_cmd_rpm": 35.0,
                "esp32_kp": 8.5,
                "esp32_ki": 48.0,
                "esp32_kd": 0.008,
            },
        ]
        if not os.path.exists(path):
            return defaults
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        jdata = data.get("joints", {})
        out = []
        for idx, jname in enumerate(self.joint_names):
            m = dict(defaults[idx])
            m.update(jdata.get(jname, {}))
            out.append(m)
        return out

    def _revisar_hot_reload(self):
        try:
            if os.path.exists(self.cal_file):
                mt = os.path.getmtime(self.cal_file)
                if mt != self.cal_mtime:
                    nueva = self._cargar_calibracion(self.cal_file)
                    with self.lock:
                        self.cfg = nueva
                    self.cal_mtime = mt
                    self._subir_pid_esp32()
                    self.get_logger().info("Calibración recargada en vivo desde hw_calibration.yaml")
        except Exception:
            pass

    def on_vel_cmd(self, msg: Float64MultiArray):
        if len(msg.data) < 3:
            return
        now = time.monotonic()
        with self.lock:
            in_settle = self.en_reposo and (now < self.settle_until)
            for i in range(3):
                c = self.cfg[i]
                vel_rad_s = float(msg.data[i])
                # Tope mínimo de seguridad para J2 (evita bajar hasta el piso)
                if i == 1 and self.pos_rad[1] <= 0.15 and vel_rad_s < 0.0:
                    self.target_rpm[i] = 0.0
                    continue
                if self.en_reposo:
                    # Durante 1.5s tras llegar, permitir solo ajuste fino HACIA ARRIBA en J2/J3
                    # (ej. terminar de subir a PH), bloqueando bajadas y J1
                    if in_settle and ((i == 1 and vel_rad_s > 0.020) or (i == 2 and vel_rad_s > 0.028)):
                        pass
                    else:
                        self.target_rpm[i] = 0.0
                        continue
                else:
                    # Limitar velocidad máxima de bajada en J2/J3 para evitar picadas
                    if i in (1, 2) and vel_rad_s < -0.20:
                        vel_rad_s = -0.20
                    # Al bajar, la gravedad ayuda: cortar antes para no pasarse
                    if ((i == 1 and abs(vel_rad_s) < 0.022) or (i == 2 and abs(vel_rad_s) < 0.048)) and vel_rad_s < 0.0:
                        self.target_rpm[i] = 0.0
                        continue
                    # Al subir (vel > 0 en J2/J3), gravedad frena al instante -> banda muerta fina (0.016)
                    if i == 1 and vel_rad_s > 0.0:
                        deadband = 0.008
                    elif i == 2 and vel_rad_s > 0.0:
                        deadband = 0.018
                    elif i == 0:
                        deadband = 0.010
                    else:
                        deadband = 0.038
                    if abs(vel_rad_s) < deadband:
                        self.target_rpm[i] = 0.0
                        continue
                ratio = float(c["counts_per_rev"]) / float(c.get("esp32_ppr", c["counts_per_rev"]))
                rpm = vel_rad_s * RADS_TO_RPM * float(c["urdf_dir"]) * ratio
                min_req = float(c["min_rpm_pos"] if rpm >= 0 else c["min_rpm_neg"])
                max_cfg = float(c.get("max_up_rpm", c["max_cmd_rpm"])) if (i == 1 and vel_rad_s > 0.0) else float(c["max_cmd_rpm"])
                lim = max(max_cfg, min_req)
                if i in (1, 2) and vel_rad_s < 0.0:
                    pass
                elif abs(rpm) < min_req:
                    rpm = math.copysign(min_req, rpm)
                self.target_rpm[i] = max(-lim, min(lim, rpm))
            self.last_cmd_time = now

    def on_estado_trayectoria(self, msg: String):
        s = msg.data.upper()
        with self.lock:
            if "EJECUTANDO" in s or "HOMING" in s:
                if self.en_reposo:
                    self.traj_start_t = time.monotonic()
                self.en_reposo = False
                self.holding = [False, False, False]
            else:
                if not self.en_reposo:
                    self.settle_until = time.monotonic() + 1.5
                self.en_reposo = True

    def on_pinza_cmd(self, msg: String):
        dato = "".join(ch for ch in msg.data if not ch.isspace())
        if not dato:
            return
        try:
            with self.ser_lock:
                self.ser.write(f"G,{dato}\n".encode("utf-8"))
            self.get_logger().info(f"Pinza enviada: G,{dato}")
        except serial.SerialException as e:
            self.get_logger().error(f"Error enviando pinza: {e}")

    def tx_loop(self):
        self._revisar_hot_reload()
        now = time.monotonic()
        with self.lock:
            if getattr(self, "fault_latched", False) or (now - self.last_cmd_time) > self.cmd_timeout:
                self.target_rpm = [0.0, 0.0, 0.0]
            in_settle = self.en_reposo and (now < getattr(self, "settle_until", 0.0))
            rpms_out = list(self.target_rpm)
            for i in range(1):
                c = self.cfg[i]
                rad_per_cnt = (2.0 * math.pi) / float(c["counts_per_rev"])
                moved = abs(self.pos_rad[i] - self.prev_pos_rad[i])
                self.prev_pos_rad[i] = self.pos_rad[i]
                j2_subiendo = (rpms_out[1] < -0.5)
                if (not self.en_reposo) and (not j2_subiendo) and abs(rpms_out[i]) >= 1.5 and moved < (1.5 * rad_per_cnt):
                    self.stall_time[i] += 0.05
                    if 0.15 <= self.stall_time[i] <= 0.65:
                        min_req = float(c["min_rpm_pos"] if rpms_out[i] >= 0 else c["min_rpm_neg"])
                        boost = max(abs(rpms_out[i]) * 1.15, min_req * 1.25, 2.8)
                        lim = float(c["max_cmd_rpm"])
                        rpms_out[i] = math.copysign(min(lim, boost), rpms_out[i])
                else:
                    self.stall_time[i] = 0.0
            # Rampa de arranque suave (1.4s) y límite de aceleración en subida de J2 (r2 < 0)
            elapsed_traj = max(0.0, now - getattr(self, "traj_start_t", 0.0))
            if elapsed_traj < 1.4:
                scale = 0.35 + 0.65 * (elapsed_traj / 1.4)
                if rpms_out[1] < 0.0:
                    rpms_out[1] *= scale
                if rpms_out[2] < 0.0:
                    rpms_out[2] *= scale
            if rpms_out[1] < self.prev_rpm_out[1] - 0.65:
                rpms_out[1] = self.prev_rpm_out[1] - 0.65
            # Reparto de corriente en Placa J1/J2: evitar pico simultáneo de J1 y subida de J2
            if rpms_out[1] < -1.0 and abs(rpms_out[0]) > 9.5:
                rpms_out[0] = math.copysign(9.5, rpms_out[0])
            if abs(rpms_out[0]) > 8.5 and rpms_out[1] < -8.5:
                rpms_out[1] = -8.5
            self.prev_rpm_out = list(rpms_out)
            r1, r2, r3 = rpms_out
            is_idle = (abs(r1) < 1e-4 and abs(r2) < 1e-4 and abs(r3) < 1e-4)

        try:
            with self.ser_lock:
                if is_idle:
                    if not self.was_idle:
                        self.ser.write(b"SYNC,0.00,0.00,0.00,0.0\n")
                        self.was_idle = True
                else:
                    self.was_idle = False
                    self.ser.write(f"SYNC,{r1:.2f},{r2:.2f},{r3:.2f},0.25\n".encode("utf-8"))
        except serial.SerialException as e:
            self.get_logger().error(f"Error serial TX: {e}")

    def serial_read_loop(self):
        alpha = 0.35
        while self.running and rclpy.ok():
            try:
                raw = self.ser.readline()
                if not raw:
                    continue
                linea = raw.decode("utf-8", "ignore").strip()
                if linea.startswith("STATE_J12,"):
                    p = linea.split(",")
                    if len(p) == 5:
                        self._update_joint(0, float(p[1]), int(p[2]), alpha)
                        self._update_joint(1, float(p[3]), int(p[4]), alpha)
                        if not self.rx_j12:
                            self.get_logger().info("Telemetría ESP32 J1/J2 conectada.")
                        self.rx_j12 = True
                elif linea.startswith("STATE_J3,"):
                    p = linea.split(",")
                    if len(p) == 3:
                        self._update_joint(2, float(p[1]), int(p[2]), alpha)
                        if not self.rx_j3:
                            self.get_logger().info("Telemetría ESP32 J3 conectada.")
                        self.rx_j3 = True
                elif linea.startswith(("WARN", "ERROR")):
                    self.get_logger().warn(f"Antena/ESP32: {linea}")
                    if "BOOT_J12" in linea or "BOOT_J3" in linea:
                        self._subir_pid_esp32()
            except Exception:
                continue

    def _update_joint(self, idx: int, rpm_raw: float, cnt_raw: int, alpha: float):
        reprogramar_pid = False
        with self.lock:
            if not hasattr(self, "last_cnt"):
                self.last_cnt = [0, 0, 0]
            prev_cnt = self.last_cnt[idx]
            now_rst = time.monotonic()
            if abs(prev_cnt) > 40 and cnt_raw == 0 and (now_rst - getattr(self, "last_rst_t", 0.0)) > 1.0:
                self.last_rst_t = now_rst
                placa = "J1/J2" if idx in (0, 1) else "J3"
                self.get_logger().error(
                    f"¡REINICIO DETECTADO EN ESP32 {placa}! Caída de voltaje (brown-out) puso encoder en 0. "
                    "Reacomoda el brazo en PH antes de rearmar."
                )
                self.fault_latched = True
                reprogramar_pid = True
            self.last_cnt[idx] = cnt_raw
            c = self.cfg[idx]
            rad_per_cnt = (2.0 * math.pi) / float(c["counts_per_rev"])
            ratio = float(c["counts_per_rev"]) / float(c.get("esp32_ppr", c["counts_per_rev"]))
            sign = float(c["urdf_dir"]) * float(c["enc_polarity"])
            v_inst = ((rpm_raw / ratio) * RPM_TO_RADS) * sign
            self.vel_rads[idx] = alpha * v_inst + (1.0 - alpha) * self.vel_rads[idx]
            self.pos_rad[idx] = self.q_init[idx] + (cnt_raw * rad_per_cnt) * sign
        if reprogramar_pid:
            self._subir_pid_esp32()

    def publish_feedback(self):
        if not (self.rx_j12 and self.rx_j3):
            now = time.monotonic()
            if now - self.last_warn_time > 2.0:
                self.get_logger().warn(
                    f"Esperando telemetría ESP32 (J1/J2={'OK' if self.rx_j12 else 'SIN RESPUESTA'}, "
                    f"J3={'OK' if self.rx_j3 else 'SIN RESPUESTA'})"
                )
                self.last_warn_time = now
            return

        with self.lock:
            q = list(self.pos_rad)
            qd = list(self.vel_rads)

        arr = Float64MultiArray()
        arr.data = q
        self.pos_pub.publish(arr)

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.joint_names
        js.position = q
        js.velocity = qd
        self.js_pub.publish(js)

    def shutdown(self):
        self.running = False
        try:
            self.ser.write(b"STOP\n")
            time.sleep(0.05)
            self.ser.close()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = ESP32RobotBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()