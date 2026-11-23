# Firmware Embebido ESP32 — Robot 2 (`miau`)

Firmware en **MicroPython** para las 3 placas ESP32 enlazadas por **ESP-NOW (Canal Wi-Fi 1)** con lazo PID cada `50 ms` (`20 Hz`).

## Archivos
- `main_ant.py` (**ESP32 Antena**): Pasarela USB-Serial (`/dev/ttyUSB0`) <-> ESP-NOW. Enruta `J1,`, `J2,`, `G,` a la placa `J1-J2` y `J3,` a la placa `J3`.
- `main_dof1_2.py` (**ESP32 J1-J2 + Pinza**): Lazo PID de velocidad con encoders (`PPR = 1060`), compensación gravitacional (`ff_up`), detección de bloqueo y servomotor de pinza (`GPIO 5`: `105°` abierto, `92°` cerrado).
- `main_dof3.py` (**ESP32 J3**): Control PID y lectura de encoder para la articulación `J3`.

## Pines GPIO (`main_dof1_2.py`)
| Actuador / Sensor | PWM / Señal | IN1 / Canal A | IN2 / Canal B |
| :--- | :--- | :--- | :--- |
| Motor `J1` + Encoder | `GPIO 26` | `GPIO 25` / `GPIO 27` | `GPIO 17` / `GPIO 33` |
| Motor `J2` + Encoder | `GPIO 32` | `GPIO 16` / `GPIO 14` | `GPIO 13` / `GPIO 19` |
| Servomotor Pinza | `GPIO 5` (50 Hz) | — | — |
