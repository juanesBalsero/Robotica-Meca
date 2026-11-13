"""Cinemática del brazo antropomórfico de 3 GDL (solo numpy, sin ROS).

Es la traducción a numpy de tu DH_local / evaluar_jacobiano_3dof de MATLAB:
    A1 = DH(q1, L1, pi/2, 0)    A2 = DH(q2, 0, 0, L2)    A3 = DH(q3, 0, 0, L3)
"""
import numpy as np


def dh(th, d, al, a):
    ct, st = np.cos(th), np.sin(th)
    ca, sa = np.cos(al), np.sin(al)
    return np.array([[ct, -st * ca,  st * sa, a * ct],
                     [st,  ct * ca, -ct * sa, a * st],
                     [0.0,      sa,       ca,      d],
                     [0.0,     0.0,      0.0,    1.0]])


class Brazo3GDL:
    def __init__(self, L1, L2, L3):
        self.L1, self.L2, self.L3 = float(L1), float(L2), float(L3)

    def fk_jacobiano(self, q):
        """Devuelve (x_e, J): posición del efector [m] y Jacobiano 3x3 [m/rad]."""
        T1 = dh(q[0], self.L1, np.pi / 2, 0.0)
        T2 = T1 @ dh(q[1], 0.0, 0.0, self.L2)
        T3 = T2 @ dh(q[2], 0.0, 0.0, self.L3)
        P0 = np.zeros(3)
        Z0 = np.array([0.0, 0.0, 1.0])
        P1, Z1 = T1[:3, 3], T1[:3, 2]
        P2, Z2 = T2[:3, 3], T2[:3, 2]
        Pe = T3[:3, 3]
        J = np.column_stack([np.cross(Z0, Pe - P0),
                             np.cross(Z1, Pe - P1),
                             np.cross(Z2, Pe - P2)])
        return Pe, J

    def fk(self, q):
        return self.fk_jacobiano(q)[0]


def resolver_dls(J, v, eps, lam_max):
    """Resuelve J q̇ = v. Lejos de singularidades usa la inversa exacta; cerca
    (σmin < eps) usa mínimos cuadrados amortiguados (Nakamura-Hanafusa).
    Devuelve (q̇, σmin, λ²)."""
    sigma_min = np.linalg.svd(J, compute_uv=False)[-1]
    if sigma_min >= eps:
        return np.linalg.solve(J, v), sigma_min, 0.0
    lam2 = (1.0 - (sigma_min / eps) ** 2) * lam_max ** 2
    qdot = J.T @ np.linalg.solve(J @ J.T + lam2 * np.eye(3), v)
    return qdot, sigma_min, lam2


def saturar_vector(qdot, vmax):
    """Si alguna articulación excede su límite, escala TODO el vector (mantiene la
    dirección del movimiento). Devuelve (q̇, factor<=1)."""
    f = float(np.max(np.abs(qdot) / vmax))
    if f > 1.0:
        return qdot / f, 1.0 / f
    return qdot, 1.0
