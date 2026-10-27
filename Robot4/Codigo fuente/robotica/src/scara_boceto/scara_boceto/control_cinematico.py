import argparse
import sys
import time
import math
import threading
from typing import List, Tuple

import numpy as np

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None


# ============================================================
# 1. GEOMETRÍA DEL SCARA (unidades SI)
# Compatible con la cinemática MATLAB compartida por el usuario.
# ============================================================

A1 = 0.130
A2 = 0.13571 + 0.00600   # 0.14171 m
D1 = 0.150
D2 = 0.060


# ============================================================
# 2. LÍMITES DE DISEÑO
# q = [theta1 (rad), theta2 (rad), d3 (m)]
# ============================================================

Q_MIN = np.array([
    np.deg2rad(-180.0),
    np.deg2rad(-180.0),
    0.000,
], dtype=float)

Q_MAX = np.array([
    np.deg2rad(180.0),
    np.deg2rad(189.0),
    0.235,
], dtype=float)

QD_MAX = np.array([
    2,    # theta1 [rad/s]
    2,    # theta2 [rad/s]
    0.15,   # d3 [m/s] = 150 mm/s
], dtype=float)

# Condición numérica máxima aceptada para el Jacobiano.
COND_MAX = 1.0e4

# Ganancia diagonal del lazo cartesiano (bloque Gain del Simulink: diag(5,10,5)).
K_DEFAULT = np.array([5.0, 10.0, 5.0], dtype=float)


# ============================================================
# 3. CINEMÁTICA DIRECTA, INVERSA Y JACOBIANO
# ============================================================

def fk(q: np.ndarray) -> np.ndarray:
    """Posición cartesiana [x,y,z] en metros."""
    th1, th2, d3 = np.asarray(q, dtype=float)
    s = th1 + th2
    return np.array([
        A1 * np.cos(th1) + A2 * np.sin(s),
        A1 * np.sin(th1) - A2 * np.cos(s),
        D1 + D2 - d3,
    ], dtype=float)


def jacobiano(q: np.ndarray) -> np.ndarray:
    """Jacobiano lineal 3x3: p_dot = J(q) @ q_dot."""
    th1, th2, _ = np.asarray(q, dtype=float)
    s = th1 + th2
    return np.array([
        [-A1 * np.sin(th1) + A2 * np.cos(s),
         A2 * np.cos(s), 0.0],
        [ A1 * np.cos(th1) + A2 * np.sin(s),
         A2 * np.sin(s), 0.0],
        [0.0, 0.0, -1.0],
    ], dtype=float)


def ik_ar(P: np.ndarray) -> np.ndarray:
    """
    Cinemática inversa para la rama usada en el MATLAB del usuario.
    Retorna [theta1, theta2, d3] en [rad, rad, m].
    """
    P = np.asarray(P, dtype=float)
    if P.shape != (3,) or not np.all(np.isfinite(P)):
        raise ValueError("El punto cartesiano debe contener x, y, z finitos.")

    x, y, z = P
    aph = math.atan2(y, x)
    d = math.hypot(x, y)

    if d < 1e-10:
        raise ValueError("Cinemática inversa indeterminada cerca del eje Z.")

    cB = (d*d + A1*A1 - A2*A2) / (2.0*d*A1)
    cphi = (A1*A1 + A2*A2 - d*d) / (2.0*A1*A2)

    # No ocultar puntos fuera del espacio de trabajo por clipping.
    tol = 1e-9
    if cB < -1.0 - tol or cB > 1.0 + tol or cphi < -1.0 - tol or cphi > 1.0 + tol:
        raise ValueError(f"Punto fuera del espacio de trabajo: {P}")

    B = math.acos(float(np.clip(cB, -1.0, 1.0)))
    phi = math.acos(float(np.clip(cphi, -1.0, 1.0)))

    th1 = aph + B
    th2 = phi - math.pi / 2.0
    d3 = D1 + D2 - z

    # Se conserva el ángulo continuo de esta rama; NO se envuelve a [-pi, pi].
    return np.array([th1, th2, d3], dtype=float)


def pinv_amortiguada(J: np.ndarray, lam: float = 1e-4) -> np.ndarray:
    """Pseudoinversa amortiguada por SVD, útil cerca de singularidades."""
    U, S, Vt = np.linalg.svd(np.asarray(J, dtype=float), full_matrices=False)
    filt = S / (S*S + lam*lam)
    return Vt.T @ np.diag(filt) @ U.T


def comprobar_q(q: np.ndarray, margen: float = 1e-8) -> None:
    """Lanza ValueError si q está fuera de los límites mecánicos."""
    q = np.asarray(q, dtype=float)
    if q.shape != (3,) or not np.all(np.isfinite(q)):
        raise ValueError("Estado articular inválido.")
    if np.any(q < Q_MIN - margen) or np.any(q > Q_MAX + margen):
        raise ValueError(
            "Límite articular excedido. "
            f"q=[{np.rad2deg(q[0]):.2f} deg, "
            f"{np.rad2deg(q[1]):.2f} deg, {q[2]*1000:.2f} mm]"
        )


# ============================================================
# 4. COMPROBACIÓN NUMÉRICA DE FACTIBILIDAD
# ============================================================

def validar_trayectoria(Pi: np.ndarray, Pf: np.ndarray,
                        n: int = 300,
                        cond_max: float = COND_MAX) -> Tuple[bool, str]:
    """
    Muestrea la recta Pi -> Pf y comprueba:
      - espacio de trabajo mediante IK;
      - límites articulares;
      - error FK/IK;
      - condición del Jacobiano.

    Es una comprobación numérica de la rama IK seleccionada, no una prueba
    analítica de factibilidad continua.
    """
    Pi = np.asarray(Pi, dtype=float)
    Pf = np.asarray(Pf, dtype=float)

    for k in range(n + 1):
        s = k / n
        P = Pi + s * (Pf - Pi)
        try:
            q = ik_ar(P)
            comprobar_q(q)
            err = np.linalg.norm(fk(q) - P)
            if err > 1e-5:
                return False, f"Error FK/IK de {err:.3e} m en P={P}."

            J = jacobiano(q)
            cond = np.linalg.cond(J)
            if not np.isfinite(cond) or cond > cond_max:
                return False, (
                    f"Singularidad/proximidad a singularidad en P={P}; "
                    f"cond(J)={cond:.3e}."
                )
        except (ValueError, np.linalg.LinAlgError) as exc:
            return False, f"Tramo no factible en P={P}: {exc}"

    return True, "Tramo factible en el muestreo."


# ============================================================
# 5. PLANIFICADOR DIFERENCIAL DE LÍNEA RECTA
# No usa perfil trapezoidal ni triangular.
# ============================================================

class PlanificadorJacobiano:
    """
    Sigue cada segmento recto usando q_dot = J(q)^(-1) * p_dot.

    La velocidad escalar solicitada se reduce automáticamente si la
    velocidad articular necesaria excede QD_MAX. Se agrega corrección
    cartesiana proporcional para reducir el error de integración.

    Nota: q es la estimación de la referencia enviada, no la medición del
    encoder, porque el firmware facilitado no transmite telemetría.
    """

    def __init__(self, Pi: np.ndarray, puntos: List[np.ndarray],
                 v_aprox: float = 0.0946, Ts: float = 0.01,
                 K=K_DEFAULT, cond_max: float = COND_MAX,
                 metodo: str = "tustin"):
        if v_aprox <= 0:
            raise ValueError("La velocidad cartesiana debe ser positiva.")
        if Ts <= 0:
            raise ValueError("Ts debe ser positivo.")
        self.Ts = float(Ts)
        self.v_aprox = float(v_aprox)
        self.K = np.diag(np.broadcast_to(np.asarray(K, dtype=float), (3,)))
        self.cond_max = float(cond_max)
        if metodo not in ("zoh", "tustin"):
            raise ValueError("metodo debe ser 'zoh' o 'tustin'.")
        self.metodo = metodo

        self.puntos = [np.asarray(Pi, dtype=float)] + [
            np.asarray(p, dtype=float) for p in puntos
        ]
        if any(p.shape != (3,) or not np.all(np.isfinite(p)) for p in self.puntos):
            raise ValueError("Cada punto debe ser un vector [x,y,z] finito.")

        self.tramos = []
        for i in range(len(self.puntos) - 1):
            A, B = self.puntos[i], self.puntos[i + 1]
            delta = B - A
            L = float(np.linalg.norm(delta))
            if L < 1e-9:
                continue
            ok, mensaje = validar_trayectoria(A, B, cond_max=self.cond_max)
            if not ok:
                raise ValueError(f"Tramo {i+1} no factible: {mensaje}")
            self.tramos.append({
                "A": A.copy(), "B": B.copy(), "L": L,
                "dir": delta / L, "s": 0.0
            })

        if not self.tramos:
            raise ValueError("La trayectoria no contiene segmentos con longitud.")

        self.indice = 0
        self.q = ik_ar(self.puntos[0])
        comprobar_q(self.q)
        self.terminado = False

    def _derivadas(self, q: np.ndarray, s: float, tramo: dict):
        """
        Ley de control del diagrama para el estado (q, s):
            q_dot = J^-1 * ( xd_dot + K * (xd - xe) )
        con xd_dot = direc * v (la magnitud v se limita por QD_MAX).
        Devuelve (q_dot, v_aplicada, x_d).
        """
        direc = tramo["dir"]
        x_d = tramo["A"] + direc * s

        J = jacobiano(q)
        cond = np.linalg.cond(J)
        if not np.isfinite(cond) or cond > self.cond_max:
            raise RuntimeError(
                f"Movimiento detenido por singularidad: cond(J)={cond:.3e}"
            )

        # Velocidad máxima cartesiana compatible con QD_MAX.
        q_por_m = np.linalg.solve(J, direc)
        v = self.v_aprox
        for j in range(3):
            if abs(q_por_m[j]) > 1e-12:
                v = min(v, QD_MAX[j] / abs(q_por_m[j]))

        e = x_d - fk(q)                              # xd - xe
        p_dot = direc * v + self.K @ e               # xd_dot + K*e
        q_dot = np.linalg.solve(J, p_dot)

        # Saturación uniforme (conserva la dirección del movimiento).
        factor = 1.0
        for j in range(3):
            if abs(q_dot[j]) > QD_MAX[j]:
                factor = min(factor, QD_MAX[j] / abs(q_dot[j]))
        return q_dot * factor, v * factor, x_d

    def paso(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray, bool, float]:
        """
        Un ciclo de Ts. Integrador 1/s discretizado con:
          zoh    : q[k+1] = q[k] + Ts*qd[k]              (= Euler adelante)
          tustin : q[k+1] = q[k] + Ts/2*(qd[k] + qd[k+1]) (implícito; se
                   resuelve con 3 iteraciones de punto fijo)
        Devuelve: q_ref, q_dot, x_d, terminado, v_aplicada [m/s].
        """
        if self.terminado:
            return self.q.copy(), np.zeros(3), self.puntos[-1].copy(), True, 0.0

        tramo = self.tramos[self.indice]
        B, L, s = tramo["B"], tramo["L"], tramo["s"]
        Ts = self.Ts

        qd0, v0, x_d = self._derivadas(self.q, s, tramo)
        dq = qd0 * Ts
        ds = v0 * Ts

        if self.metodo == "tustin":
            q_n = self.q + dq
            s_n = min(L, s + ds)
            for _ in range(3):
                qd1, v1, _ = self._derivadas(q_n, s_n, tramo)
                q_n = self.q + 0.5 * Ts * (qd0 + qd1)
                s_n = min(L, s + 0.5 * Ts * (v0 + v1))
            dq = q_n - self.q
            ds = s_n - s

        # Impedir que la integración cruce límites mecánicos.
        factor = 1.0
        for j in range(3):
            if dq[j] > 0:
                factor = min(factor, max(0.0, (Q_MAX[j] - self.q[j]) / dq[j]))
            elif dq[j] < 0:
                factor = min(factor, max(0.0, (Q_MIN[j] - self.q[j]) / dq[j]))
        dq = dq * factor
        ds = ds * factor
        q_dot = dq / Ts
        v_aplicada = ds / Ts

        if np.max(np.abs(q_dot)) < 1e-10 and s < L - 1e-6:
            raise RuntimeError(
                "Trayectoria bloqueada por los límites articulares."
            )

        q_nueva = self.q + dq
        if np.any(q_nueva < Q_MIN - 1e-7) or np.any(q_nueva > Q_MAX + 1e-7):
            raise RuntimeError("El siguiente comando excedería un límite articular.")
        self.q = np.clip(q_nueva, Q_MIN, Q_MAX)
        tramo["s"] = min(L, s + ds)

        # Cuando se alcanza el extremo, continuar corrigiendo el pequeño
        # error cartesiano antes de pasar al siguiente segmento.
        x_actual = fk(self.q)
        if tramo["s"] >= L - 1e-9 and np.linalg.norm(B - x_actual) < 5e-4:
            self.q = ik_ar(B)
            comprobar_q(self.q)
            self.indice += 1
            if self.indice >= len(self.tramos):
                self.terminado = True
                return self.q.copy(), np.zeros(3), B.copy(), True, 0.0

        return self.q.copy(), q_dot.copy(), x_d.copy(), False, v_aplicada


# ============================================================
# 6. COMUNICACIÓN USB-SERIAL CON ESP32
# ============================================================

def listar_puertos():
    if list_ports is None:
        print("Instala pyserial: pip install pyserial")
        return
    puertos = list(list_ports.comports())
    if not puertos:
        print("No se detectaron puertos serie.")
    for p in puertos:
        print(f"{p.device}: {p.description}")


class EnlaceESP32:
    def __init__(self, puerto: str, baudrate: int = 115200,
                 timeout: float = 0.02):
        if serial is None:
            raise RuntimeError("Falta pyserial. Instala con: pip install pyserial")
        self.ser = serial.Serial(
            port=puerto, baudrate=baudrate, timeout=timeout,
            write_timeout=0.2
        )
        # Algunas placas reinician al abrir el puerto.
        time.sleep(2.0)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self.lock = threading.Lock()

    def enviar_posicion(self, q: np.ndarray):
        """Convierte [rad, rad, m] a grados, grados y mm para el firmware."""
        q = np.asarray(q, dtype=float)
        comprobar_q(q)
        th1_deg = np.rad2deg(q[0])
        th2_deg = np.rad2deg(q[1])
        d3_mm = q[2] * 1000.0
        linea = f"{th1_deg:.4f} {th2_deg:.4f} {d3_mm:.3f}\n"
        with self.lock:
            self.ser.write(linea.encode("ascii"))
            self.ser.flush()

    def enviar_home_actual(self):
        """Pone a cero los contadores según el comando 999 del firmware."""
        with self.lock:
            self.ser.write(b"999\n")
            self.ser.flush()

    def leer_mensajes(self):
        """Lee y muestra líneas disponibles de la ESP32 sin bloquear el control."""
        if self.ser.in_waiting:
            linea = self.ser.readline().decode("utf-8", errors="replace").strip()
            if linea:
                print(f"[ESP32] {linea}")

    def cerrar(self):
        if self.ser and self.ser.is_open:
            self.ser.close()


# ============================================================
# 7. EJECUCIÓN
# ============================================================

def parse_punto(texto: str) -> np.ndarray:
    vals = [float(v.strip()) for v in texto.split(",")]
    if len(vals) != 3:
        raise argparse.ArgumentTypeError("Usa x,y,z en metros.")
    return np.array(vals, dtype=float)


def parse_puntos(texto: str) -> List[np.ndarray]:
    # Varios puntos separados por punto y coma: x,y,z;x,y,z
    try:
        return [parse_punto(p.strip()) for p in texto.split(";") if p.strip()]
    except argparse.ArgumentTypeError as exc:
        raise argparse.ArgumentTypeError(str(exc)) from exc


def main():
    parser = argparse.ArgumentParser(
        description="SCARA RRP: Jacobiano inverso + USB Serial ESP32."
    )
    parser.add_argument("--port", default="/dev/ttyACM0",
                        help="Puerto serie, por ejemplo COM5 o /dev/ttyUSB0.")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--listar-puertos", action="store_true")
    parser.add_argument("--inicio", type=parse_punto,
                        default=parse_punto("0.13,-0.14171,0.15362"),
                        help="Punto inicial x,y,z en metros.")
    parser.add_argument("--puntos", type=parse_puntos,
                        default=parse_puntos("-0.14279,-0.08409,0.10116"),
                        help="Puntos destino separados por ';', coordenadas en metros.")
    parser.add_argument("--Ts", type=float, default=0.01,
                        help="Periodo del planificador, segundos (default 0.01).")
    parser.add_argument("--v-aprox", type=float, default=0.01,
                        help="Velocidad cartesiana nominal en m/s.")
    parser.add_argument("--tiempo", type=float, default=None,
                        help="Tiempo total deseado [s]. Recalcula la magnitud de "
                             "velocidad: v = longitud_total / tiempo (anula --v-aprox).")
    parser.add_argument("--K", type=parse_punto, default=K_DEFAULT,
                        help="Ganancia diagonal del lazo, 'kx,ky,kz' (default 5,10,5).")
    parser.add_argument("--metodo", choices=["zoh", "tustin"], default="tustin",
                        help="Discretización del integrador 1/s (default tustin).")
    parser.add_argument("--cond-max", type=float, default=COND_MAX)
    parser.add_argument("--simular", action="store_true",
                        help="Simula sin enviar comandos al puerto serie.")
    parser.add_argument("--home-actual", action="store_true",
                        help="Envía 999. Úsalo solo si la posición física actual debe ser el cero.")
    parser.add_argument("--pausa-inicial", type=float, default=1.0,
                        help="Espera tras abrir el puerto antes de mover.")
    args = parser.parse_args()

    if args.listar_puertos:
        listar_puertos()
        return

    if not args.simular and not args.port:
        parser.error("Indica --port COMx o /dev/ttyUSBx, o utiliza --listar-puertos.")

    if args.Ts <= 0 or args.v_aprox <= 0:
        parser.error("--Ts y --v-aprox deben ser positivos.")

    if args.tiempo is not None:
        if args.tiempo <= 0:
            parser.error("--tiempo debe ser positivo.")
        pts = [args.inicio] + list(args.puntos)
        L_total = sum(float(np.linalg.norm(pts[i + 1] - pts[i]))
                      for i in range(len(pts) - 1))
        args.v_aprox = L_total / args.tiempo
        print(f"Longitud total = {L_total:.5f} m -> v-aprox = {args.v_aprox:.5f} m/s")

    try:
        planner = PlanificadorJacobiano(
            Pi=args.inicio,
            puntos=args.puntos,
            v_aprox=args.v_aprox,
            Ts=args.Ts,
            K=args.K,
            cond_max=args.cond_max,
            metodo=args.metodo,
        )
    except (ValueError, np.linalg.LinAlgError) as exc:
        print(f"[ERROR] Trayectoria no válida: {exc}", file=sys.stderr)
        sys.exit(2)

    print("\n=== Planificador SCARA RRP ===")
    print(f"Inicio: {args.inicio} m")
    for i, p in enumerate(args.puntos, start=1):
        print(f"Destino {i}: {p} m")
    print("Límites q_min:", Q_MIN, "[rad, rad, m]")
    print("Límites q_max:", Q_MAX, "[rad, rad, m]")
    print("Velocidades máximas:", QD_MAX, "[rad/s, rad/s, m/s]")
    print("q inicial:", planner.q)
    print("Posición FK inicial:", fk(planner.q))

    enlace = None
    try:
        if not args.simular:
            enlace = EnlaceESP32(args.port, args.baud)
            print(f"Conectado a {args.port} a {args.baud} baudios.")
            if args.home_actual:
                print("ATENCIÓN: enviando 999; se toma la posición física actual como cero.")
                enlace.enviar_home_actual()
                time.sleep(0.2)
            time.sleep(max(0.0, args.pausa_inicial))

            # Enviar la consigna inicial una vez; no se ejecuta homing automático.
            enlace.enviar_posicion(planner.q)

        t_sig = time.monotonic()
        n = 0
        while True:
            t_sig += args.Ts
            q_ref, q_dot, x_d, fin, v_aplicada = planner.paso()

            if enlace is not None:
                enlace.enviar_posicion(q_ref)
                enlace.leer_mensajes()

            if n % max(1, round(0.5 / args.Ts)) == 0:
                p_estim = fk(q_ref)
                print(
                    f"q=[{np.rad2deg(q_ref[0]):7.2f}°, "
                    f"{np.rad2deg(q_ref[1]):7.2f}°, {q_ref[2]*1000:7.2f} mm] | "
                    f"|qdot|=[{abs(q_dot[0]):.3f}, {abs(q_dot[1]):.3f}, "
                    f"{abs(q_dot[2]):.3f}] | v={v_aplicada:.4f} m/s | "
                    f"P_est={p_estim}"
                )

            n += 1
            if fin:
                # Mantener la consigna final. La ESP32 conserva su lazo de posición.
                if enlace is not None:
                    for _ in range(5):
                        enlace.enviar_posicion(q_ref)
                        time.sleep(args.Ts)
                print(f"Trayectoria completada (referencia calculada) en "
                      f"{n * args.Ts:.2f} s.")
                break

            if not args.simular:
                restante = t_sig - time.monotonic()
                if restante > 0:
                    time.sleep(restante)
                else:
                    # Si se atrasa, no acumular una cola de comandos.
                    t_sig = time.monotonic()

    except KeyboardInterrupt:
        print("\nParada solicitada por el usuario.")
        if enlace is not None:
            # Mantener última referencia: no inventar una posición de parada.
            print("Se conserva la última consigna; usa la parada segura de tu sistema.")
    except (RuntimeError, ValueError, OSError) as exc:
        print(f"\n[ERROR] Movimiento interrumpido: {exc}", file=sys.stderr)
        print("No se enviarán más referencias. Activa la parada segura del sistema.")
        raise SystemExit(1)
    finally:
        if enlace is not None:
            enlace.cerrar()


if __name__ == "__main__":
    main()