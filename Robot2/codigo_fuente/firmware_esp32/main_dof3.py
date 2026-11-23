from machine import Pin, PWM
import machine, network, espnow, time

MAC_ANTENA = b'\xb4\xbf\xe9\x11v\x9c'
PPR = 530               # Pulsos por revolución de J3 (hw_calibration.yaml)
PERIODO_MS = 50         # Lazo de control a 20 Hz
ALPHA = 0.5
MAX_DUTY_STEP = 185

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
        self.half_steps += (-1 if b else 1) if a else (1 if b else -1)
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
        if (duty > 0 and self.last_duty < 0) or (duty < 0 and self.last_duty > 0):
            duty = 0
        else:
            diff = duty - self.last_duty
            if abs(diff) > MAX_DUTY_STEP:
                duty = self.last_duty + (MAX_DUTY_STEP if diff > 0 else -MAX_DUTY_STEP)
        self.last_duty = duty
        self.in1.value(1 if duty > 0 else 0)
        self.in2.value(1 if duty < 0 else 0)
        self.pwm.duty(abs(duty))

    def brake(self):
        self.last_duty = 0
        self.in1.value(1); self.in2.value(1); self.pwm.duty(1023)

    def stop(self):
        self.last_duty = 0
        self.in1.value(0); self.in2.value(0); self.pwm.duty(0)

class PID:
    def __init__(self, kp, ki, kd):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.integ = 0.0; self.prev_err = 0.0

    def reset(self):
        self.integ = 0.0; self.prev_err = 0.0

    def compute(self, sp, meas, dt):
        err = sp - meas
        p = self.kp * err
        d = self.kd * (err - self.prev_err) / dt if dt > 0 else 0.0
        self.prev_err = err
        if -950.0 < (p + d + self.ki * self.integ) < 950.0:
            self.integ += err * dt
        return max(-950.0, min(950.0, p + self.ki * self.integ + d))

# Pines GPIO placa ESP32 J3 (PWM=26, IN1=25, IN2=17, ENC_A=27, ENC_B=33)
joint_j3 = MotorJoint(26, 25, 17, 27, 33, 10.4, 68.0, 0.012)
cfg_j3 = {"ff_up": 318.0, "ff_dn": 265.0}

def aplicar_cfg(cmd, pid=None):
    # Formato: J3,CFG,kp,ki,kd,ff_up,ff_dn
    p = cmd.split(",")
    if len(p) >= 7 and p[0] == "J3" and p[1] == "CFG":
        joint_j3.kp, joint_j3.ki, joint_j3.kd = float(p[2]), float(p[3]), float(p[4])
        cfg_j3["ff_up"], cfg_j3["ff_dn"] = float(p[5]), float(p[6])
        if pid:
            pid.kp, pid.ki, pid.kd = joint_j3.kp, joint_j3.ki, joint_j3.kd

def control_sincrono(sp_rpm, duracion_s):
    dt_nom = PERIODO_MS / 1000.0
    pid = PID(joint_j3.kp, joint_j3.ki, joint_j3.kd)
    last_cnt, rpm_f, prev_cmd, stall_ticks, stall_ref = 0, 0.0, 0.0, 0, 0
    joint_j3.enc.reset(); joint_j3.brake()
    last_t = t_ini = time.ticks_ms()
    try:
        while True:
            while True:
                _, rx = e.irecv(0)
                if not rx: break
                cmd = rx.decode("utf-8", "ignore").strip()
                if cmd == "STOP": return
                elif cmd.startswith("J3,CFG,"): aplicar_cfg(cmd, pid)
                elif cmd.startswith("SYNC,"):
                    p = cmd.split(",")
                    if len(p) == 5:
                        sp_rpm, duracion_s, t_ini = float(p[3]), float(p[4]), time.ticks_ms()
            now = time.ticks_ms()
            if duracion_s > 0 and time.ticks_diff(now, t_ini) / 1000.0 >= duracion_s:
                sp_rpm = 0.0
            dt = time.ticks_diff(now, last_t) / 1000.0
            if dt < dt_nom:
                time.sleep_ms(2); continue
            last_t = now
            cnt = joint_j3.enc.count
            rpm_c = ((cnt - last_cnt) / PPR) * (60.0 / dt)
            last_cnt = cnt
            rpm_f = ALPHA * rpm_c + (1.0 - ALPHA) * rpm_f
            if abs(sp_rpm) < 0.05:
                pid.reset(); stall_ticks = 0; joint_j3.brake()
            else:
                if (sp_rpm > 0) != (prev_cmd > 0) or abs(prev_cmd) < 0.05:
                    pid.reset(); stall_ticks = 0; stall_ref = cnt
                duty = pid.compute(sp_rpm, rpm_f, dt)
                if sp_rpm < 0:
                    duty = max(-950.0, min(0.0, duty - cfg_j3["ff_up"]))
                else:
                    duty = max(0.0, min(950.0, duty + cfg_j3["ff_dn"]))
                if abs(duty) >= 550 and abs(cnt - stall_ref) < 3:
                    stall_ticks += 1
                    if stall_ticks >= 20:
                        tx_pc("ERROR_ENC_J3,Sin_pulsos"); return
                else:
                    stall_ticks = 0; stall_ref = cnt
                joint_j3.set_duty(duty)
            prev_cmd = sp_rpm
            tx_pc(f"STATE_J3,{rpm_f:.2f},{cnt}")
    finally:
        joint_j3.stop()

joint_j3.stop()
tx_pc(f"WARN,BOOT_J3,cause={machine.reset_cause()}")
print("LISTA_ESP32_J3")
while True:
    _, msg = e.irecv(50)
    if not msg: continue
    cmd = msg.decode("utf-8", "ignore").strip()
    if cmd == "STOP": joint_j3.stop()
    elif cmd.startswith("J3,CFG,"): aplicar_cfg(cmd)
    elif cmd.startswith("SYNC,"):
        p = cmd.split(",")
        if len(p) == 5:
            try: control_sincrono(float(p[3]), float(p[4]))
            except Exception as err: joint_j3.stop(); tx_pc(f"ERROR_J3,{err}")
