#!/usr/bin/env python3

import csv
import math
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


JOINTS = ('M1', 'M2', 'M3')
REQUIRED_COLUMNS = (
    't_s', 'q1_rad', 'q2_rad', 'M3_m',
    'q1_rad_s', 'q2_rad_s', 'M3_m_s',
)
POSITION_LIMITS = ((-3.1416, 3.1416), (-2.8, 2.8), (0.0, 0.05))
VELOCITY_LIMITS = (2.0, 2.0, 0.05)
# La trayectoria calibrada con el TCP real usa hasta aproximadamente -1.707.
# Se deja margen hasta -1.80, sin habilitar el plegado extremo cercano a -2.1
# que anteriormente produjo solapamiento visual entre los eslabones.
SAFE_M2_MIN = -1.8
# La barra superior de la T de L4 comienza a entrar en la carcasa de L3
# cerca de 0.022 m. Este límite conserva aproximadamente 2.5 mm de holgura
# y también mantiene los dedos por encima de la mesa.
SAFE_M3_TASK_MAX = 0.020
APPROACH_TIME = 10.0


def duration_from_seconds(seconds):
    total_nanoseconds = int(round(seconds * 1_000_000_000))
    duration = rclpy.duration.Duration(nanoseconds=total_nanoseconds)
    return duration.to_msg()


class CsvTrajectoryRunner(Node):
    def __init__(self):
        super().__init__('ejecutar_trayectoria_csv')
        self._current_positions = None
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/brazo_controller/follow_joint_trajectory',
        )
        self.create_subscription(JointState, '/joint_states', self._joint_state_cb, 10)

    def _joint_state_cb(self, message):
        positions = dict(zip(message.name, message.position))
        if all(joint in positions for joint in JOINTS):
            self._current_positions = [positions[joint] for joint in JOINTS]

    def wait_for_robot(self, timeout=15.0):
        self.get_logger().info('Esperando el controlador y /joint_states...')
        if not self._action_client.wait_for_server(timeout_sec=timeout):
            raise RuntimeError(
                'No aparece /brazo_controller/follow_joint_trajectory. '
                'Comprueba que Gazebo y los controladores estén activos.'
            )

        deadline = time.monotonic() + timeout
        while self._current_positions is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
        if self._current_positions is None:
            raise RuntimeError('No se recibió /joint_states en 15 segundos.')

    def send_points(self, points, description):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        goal.trajectory.points = points

        self.get_logger().info(description)
        send_future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError('El controlador rechazó la trayectoria.')

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapped_result = result_future.result()
        if wrapped_result is None:
            raise RuntimeError('No se recibió resultado del controlador.')

        result = wrapped_result.result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(
                f'Trayectoria abortada: código {result.error_code}; '
                f'{result.error_string}'
            )

    def execute(self, rows):
        first = rows[0]
        target = [first['q1_rad'], first['q2_rad'], first['M3_m']]

        approach = JointTrajectoryPoint()
        approach.positions = target
        approach.velocities = [0.0, 0.0, 0.0]
        approach.time_from_start = duration_from_seconds(APPROACH_TIME)
        self.send_points(
            [approach],
            'Aproximando el robot al primer punto durante 10.0 s...',
        )

        trajectory_points = []
        for row in rows[1:]:
            point = JointTrajectoryPoint()
            point.positions = [row['q1_rad'], row['q2_rad'], row['M3_m']]
            point.velocities = [
                row['q1_rad_s'], row['q2_rad_s'], row['M3_m_s']
            ]
            point.time_from_start = duration_from_seconds(row['t_s'])
            trajectory_points.append(point)

        self.send_points(
            trajectory_points,
            f'Ejecutando {len(rows)} muestras del CSV durante '
            f'{rows[-1]["t_s"]:.2f} s...',
        )
        self.get_logger().info('Trayectoria CSV completada correctamente.')


def load_and_validate_csv(csv_path):
    with csv_path.open(newline='', encoding='utf-8-sig') as csv_file:
        reader = csv.DictReader(csv_file)
        missing = [name for name in REQUIRED_COLUMNS if name not in (reader.fieldnames or [])]
        if missing:
            raise ValueError(f'Faltan columnas en el CSV: {missing}')

        rows = []
        for line_number, raw in enumerate(reader, start=2):
            try:
                row = {name: float(raw[name]) for name in REQUIRED_COLUMNS}
            except (TypeError, ValueError) as error:
                raise ValueError(f'Dato no numérico en la línea {line_number}.') from error
            if not all(math.isfinite(value) for value in row.values()):
                raise ValueError(f'Dato no finito en la línea {line_number}.')
            rows.append(row)

    if len(rows) < 2:
        raise ValueError('El CSV debe contener al menos dos muestras.')
    if abs(rows[0]['t_s']) > 1e-9:
        raise ValueError('La primera muestra debe comenzar en t_s=0.')

    previous_time = -1.0
    for index, row in enumerate(rows, start=1):
        if row['t_s'] <= previous_time:
            raise ValueError(f't_s no es creciente en la muestra {index}.')
        previous_time = row['t_s']

        positions = (row['q1_rad'], row['q2_rad'], row['M3_m'])
        velocities = (row['q1_rad_s'], row['q2_rad_s'], row['M3_m_s'])
        if row['q2_rad'] < SAFE_M2_MIN:
            raise ValueError(
                f'M2={row["q2_rad"]:.6f} entra en la zona de '
                f'autocolisión (< {SAFE_M2_MIN}) en muestra {index}.'
            )
        if row['M3_m'] > SAFE_M3_TASK_MAX + 1e-9:
            raise ValueError(
                f'M3={row["M3_m"]:.6f} atravesaría la mesa '
                f'(máximo seguro {SAFE_M3_TASK_MAX}) en muestra {index}.'
            )
        for joint, position, limits in zip(JOINTS, positions, POSITION_LIMITS):
            if not limits[0] <= position <= limits[1]:
                raise ValueError(
                    f'{joint}={position:.6f} fuera de {limits} en muestra {index}.'
                )
        for joint, velocity, limit in zip(JOINTS, velocities, VELOCITY_LIMITS):
            if abs(velocity) > limit + 1e-6:
                raise ValueError(
                    f'Velocidad de {joint}={velocity:.6f} supera {limit} '
                    f'en muestra {index}.'
                )
    return rows


def main(args=None):
    rclpy.init(args=args)
    node = CsvTrajectoryRunner()
    try:
        csv_path = Path(get_package_share_directory('brazo_estesi')) / (
            'trajectories/referencias_propuesta.csv'
        )
        node.get_logger().info(f'Leyendo: {csv_path}')
        rows = load_and_validate_csv(csv_path)
        node.get_logger().info(
            f'CSV válido: {len(rows)} muestras, duración {rows[-1]["t_s"]:.2f} s.'
        )
        node.wait_for_robot()
        node.execute(rows)
    except (OSError, ValueError, RuntimeError) as error:
        node.get_logger().error(str(error))
        raise SystemExit(1) from error
    except KeyboardInterrupt:
        node.get_logger().warning('Ejecución interrumpida por el usuario.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
