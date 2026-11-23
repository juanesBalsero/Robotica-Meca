from machine import Pin, PWM
import machine
import network
import espnow
import time

MAC_ANTENA = b'\xb4\xbf\xe9\x11v\x9c'
PPR = 1060
PERIODO_MS = 50
ALPHA = 0.5

PIN_GRIPPER = 5
ANG_ABIERTO = 105    # Ligeramente menor a 110 para no forzar el tope mecánico
ANG_CERRADO = 92
MAX_DUTY_STEP = 185   # Salto máximo de PWM por ciclo (50 ms) para evitar picos de corriente

wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.disconnect()
wlan.config(channel=1)

e = espnow.ESPNow()
e.active(True)
e.add_peer(MAC_ANTENA)


def tx_pc(texto):
    try:
        e.send(MAC_ANTENA, texto.encode("utf-8"), False)
    except OSError:
        pass


class GripperServo:
    def __init__(self, pin_num, ang_open=105, ang_close=92):
        self.pwm = PWM(Pin(pin_num), freq=50)
        self.ang_open = ang_open
        self.ang_close = ang_close
        self.angulo = ang_open
        self.detach_at = None
        self.set_angle(ang_open, auto_detach_ms=450)

    def set_angle(self, deg, auto_detach_ms=None):
        deg = max(0.0, min(180.0, float(deg)))
        us = 500.0 + (deg / 180.0) * 2000.0
        duty = int((us / 20000.0) * 1023)
        self.pwm.duty(duty)
        self.angulo = deg
        if auto_detach_ms is not None:
            self.detach_at = time.ticks_add(time.ticks_ms(), auto_detach_ms)
        else:
            self.detach_at = None

    def update(self):
        # Apaga el PWM de forma no bloqueante tras abrir para eliminar consumo en reposo
        if self.detach_at is not None and time.ticks_diff(time.ticks_ms(), self.detach_at) >= 0:
            self.pwm.duty(0)
            self.detach_at = None

    def procesar(self, cmd):
        partes = cmd.split(",", 1)
        if len(partes) != 2:
            return
        val = partes[1].strip().upper()
        if val in ("ABRIR", "OPEN"):
            self.set_angle(self.ang_open, auto_detach_ms=450)
        elif val in ("CERRAR", "CLOSE"):
            # Mantiene el pulso activo al cerrar para sujetar la caja con firmeza
            self.set_angle(self.ang_close, auto_detach_ms=None)
        else:
            self.set_angle(float(val), auto_detach_ms=450)
        tx_pc(f"GRIP_OK,{self.angulo:.1f}")


class VelEncoder:
    def __init__(self, pin_a, pin_b):
        self.pin_a = Pin(pin_a, Pin.IN, Pin.PULL_UP)
        self.pin_b = Pin(pin_b, Pin.IN, Pin.PULL_UP)
        self.last_a = self.pin_a.value()
        self.half_steps = 0
        self.count = 0
        self.pin_a.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self._isr)

    def reset(self):
        self.last_a = self.pin_a.value()
        self.half_steps = 0
        self.count = 0

    def _isr(self, pin):
        a = (self.pin_a.value() + self.pin_a.value() + self.pin_a.value()) >= 2
        if a == self.last_a:
            return
        b = (self.pin_b.value() + self.pin_b.value() + self.pin_b.value()) >= 2
        self.last_a = a
        if a:
            self.half_steps += -1 if b else 1
        else:
            self.half_steps += 1 if b else -1
        self.count = int(self.half_steps / 2)


class MotorJoint:
    def __init__(self, pin_pwm, pin_in1, pin_in2, pin_enc_a, pin_enc_b, kp, ki, kd):
        self.pwm = PWM(Pin(pin_pwm), freq=2000)
        self.in1 = Pin(pin_in1, Pin.OUT)
        self.in2 = Pin(pin_in2, Pin.OUT)
        self.enc = VelEncoder(pin_enc_a, pin_enc_b)
        self.kp, self.ki, self.kd = kp, ki, kd
        self.last_duty = 0
        self.stop()

    def set_duty(self, duty):
        duty = int(max(-950, min(950, duty)))
        # Si cambia de sentido, pasa primero por 0 para proteger el puente H y evitar picos
        if (duty > 0 and self.last_duty < 0) or (duty < 0 and self.last_duty > 0):
            duty = 0
        else:
            diff = duty - self.last_duty
            if abs(diff) > MAX_DUTY_STEP:
                duty = self.last_duty + (MAX_DUTY_STEP if diff > 0 else -MAX_DUTY_STEP)
        self.last_duty = duty

        if duty == 0:
            self.in1.value(0)
            self.in2.value(0)
            self.pwm.duty(0)
        elif duty > 0:
            self.in1.value(1)
            self.in2.value(0)
            self.pwm.duty(duty)
        else:
            self.in1.value(0)
            self.in2.value(1)
            self.pwm.duty(-duty)

    def brake(self):
        self.last_duty = 0
        self.in1.value(1)
        self.in2.value(1)
        self.pwm.duty(1023)

    def stop(self):
        self.last_duty = 0
        self.in1.value(0)
        self.in2.value(0)
        self.pwm.duty(0)


class PID:
    def __init__(self, kp, ki, kd):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.integ = 0.0
        self.prev_err = 0.0

    def reset(self):
        self.integ = 0.0
        self.prev_err = 0.0

    def compute(self, sp, meas, dt):
        err = sp - meas
        p = self.kp * err
        d = self.kd * (err - self.prev_err) / dt if dt > 0 else 0.0
        self.prev_err = err
        if -950.0 < (p + d + self.ki * self.integ) < 950.0:
            self.integ += err * dt
        return max(-950.0, min(950.0, p + self.ki * self.integ + d))


joints = {
    "J1": MotorJoint(26, 25, 17, 27, 33, 12.5, 88.0, 0.015),
    "J2": MotorJoint(32, 16, 13, 14, 19, 8.5, 74.0, 0.018),
}
cfg_j2 = {"ff_up": 450.0, "start_boost": 35.0, "max_dn": 362.0}
gripper = GripperServo(PIN_GRIPPER, ANG_ABIERTO, ANG_CERRADO)


def aplicar_cfg(cmd, pids=None):
    # Formato: J1,CFG,kp,ki,kd  o  J2,CFG,kp,ki,kd,ff_up,start_boost,max_dn
    p = cmd.split(",")
    if len(p) >= 5 and p[0] in joints and p[1] == "CFG":
        k = p[0]
        joints[k].kp = float(p[2])
        joints[k].ki = float(p[3])
        joints[k].kd = float(p[4])
        if pids and k in pids:
            pids[k].kp = joints[k].kp
            pids[k].ki = joints[k].ki
            pids[k].kd = joints[k].kd
        if k == "J2" and len(p) >= 8:
            cfg_j2["ff_up"] = float(p[5])
            cfg_j2["start_boost"] = float(p[6])
            cfg_j2["max_dn"] = float(p[7])


def stop_all():
    for j in joints.values():
        j.stop()


def control_sincrono(rpms, duracion_s):
    dt_nom = PERIODO_MS / 1000.0
    pids = {k: PID(j.kp, j.ki, j.kd) for k, j in joints.items()}
    last_cnt = {"J1": 0, "J2": 0}
    rpm_f = {"J1": 0.0, "J2": 0.0}
    prev_cmd = {"J1": 0.0, "J2": 0.0}
    stall_ticks = {"J1": 0, "J2": 0}
    stall_ref_cnt = {"J1": 0, "J2": 0}

    for j in joints.values():
        j.enc.reset()
        j.brake()

    last_t = time.ticks_ms()
    t_ini = last_t
    salir_stop = False

    try:
        while True:
            # Vacía todos los paquetes acumulados en ESP-NOW para evitar buffer error
            while True:
                _, rx = e.irecv(0)
                if not rx:
                    break
                cmd = rx.decode("utf-8", "ignore").strip()
                if cmd == "STOP":
                    salir_stop = True
                    break
                elif cmd.startswith("G,"):
                    gripper.procesar(cmd)
                elif ",CFG," in cmd:
                    aplicar_cfg(cmd, pids)
                elif cmd.startswith("SYNC,"):
                    p = cmd.split(",")
                    if len(p) == 5:
                        rpms["J1"] = float(p[1])
                        rpms["J2"] = float(p[2])
                        duracion_s = float(p[4])
                        t_ini = time.ticks_ms()

            if salir_stop:
                break

            gripper.update()

            now = time.ticks_ms()
            if duracion_s > 0 and time.ticks_diff(now, t_ini) / 1000.0 >= duracion_s:
                rpms["J1"] = 0.0
                rpms["J2"] = 0.0

            dt = time.ticks_diff(now, last_t) / 1000.0
            if dt < dt_nom:
                time.sleep_ms(2)
                continue
            last_t = now

            estado = []
            fallo_enc = False
            for k in ("J1", "J2"):
                j = joints[k]
                cnt = j.enc.count
                rpm_c = ((cnt - last_cnt[k]) / PPR) * (60.0 / dt)
                last_cnt[k] = cnt
                rpm_f[k] = ALPHA * rpm_c + (1.0 - ALPHA) * rpm_f[k]
                sp = rpms[k]

                if abs(sp) < 0.05:
                    pids[k].reset()
                    stall_ticks[k] = 0
                    j.brake()
                else:
                    if (sp > 0) != (prev_cmd[k] > 0) or abs(prev_cmd[k]) < 0.05:
                        pids[k].reset()
                        stall_ticks[k] = 0
                        stall_ref_cnt[k] = cnt

                    duty = pids[k].compute(sp, rpm_f[k], dt)
                    if k == "J2":
                        if sp < 0:
                            boost = -cfg_j2["start_boost"] if abs(rpm_f[k]) < 0.5 else 0.0
                            duty = max(-950.0, min(0.0, duty - cfg_j2["ff_up"] + boost))
                        else:
                            duty = max(-450.0, min(cfg_j2["max_dn"], duty))

                    if abs(duty) >= 550:
                        if abs(cnt - stall_ref_cnt[k]) < 3:
                            stall_ticks[k] += 1
                            if stall_ticks[k] >= 20:
                                tx_pc(f"ERROR_ENC_{k},Sin_pulsos")
                                fallo_enc = True
                                break
                        else:
                            stall_ticks[k] = 0
                            stall_ref_cnt[k] = cnt
                    else:
                        stall_ticks[k] = 0
                        stall_ref_cnt[k] = cnt

                    j.set_duty(duty)

                prev_cmd[k] = sp
                estado.append(f"{rpm_f[k]:.2f},{cnt}")

            if fallo_enc:
                break

            tx_pc("STATE_J12," + ",".join(estado))
    finally:
        stop_all()


stop_all()
tx_pc(f"WARN,BOOT_J12,cause={machine.reset_cause()}")
print("LISTA_ESP32_J1_J2")

while True:
    gripper.update()
    _, msg = e.irecv(50)
    if not msg:
        continue
    cmd = msg.decode("utf-8", "ignore").strip()
    if cmd == "STOP":
        stop_all()
    elif ",CFG," in cmd:
        aplicar_cfg(cmd)
    elif cmd.startswith("G,"):
        try:
            gripper.procesar(cmd)
        except Exception as err:
            tx_pc(f"ERROR_GRIP,{err}")
    elif cmd.startswith("SYNC,"):
        p = cmd.split(",")
        if len(p) == 5:
            try:
                control_sincrono({"J1": float(p[1]), "J2": float(p[2])}, float(p[4]))
            except Exception as err:
                stop_all()
                tx_pc(f"ERROR_J12,{err}")