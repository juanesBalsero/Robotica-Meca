import network
import espnow
import sys
import select
import time

# Direcciones MAC de las ESP32 montadas en el manipulador
MAC_ROBOT_J12 = b'\xb4\xbf\xe9\r\x8f`'      # Placa 1: articulaciones J1, J2 y gripper (GPIO 5)
MAC_ROBOT_J3  = b'\xb4\xbf\xe9\x11\x9b\xd8'  # Placa 2: articulación J3

wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.disconnect()
wlan.config(channel=1)

e = espnow.ESPNow()
e.active(True)
e.add_peer(MAC_ROBOT_J12)
e.add_peer(MAC_ROBOT_J3)

def enviar_peer(mac_destino, payload_bytes):
    try:
        e.send(mac_destino, payload_bytes, True)
    except OSError as err:
        print(f"WARN,Fallo_ESPNOW_{mac_destino[-2:].hex()}:{err}")

poll = select.poll()
poll.register(sys.stdin, select.POLLIN)
buffer_usb = ""

print("ANTENA_LISTA")

while True:
    # PC (USB) -> Robot (ESP-NOW)
    if poll.poll(0):
        ch = sys.stdin.read(1)
        if ch == "\n":
            cmd = buffer_usb.strip()
            buffer_usb = ""
            if cmd:
                data = cmd.encode("utf-8")
                if cmd.startswith(("J1,", "J2,", "G,")):
                    enviar_peer(MAC_ROBOT_J12, data)
                elif cmd.startswith("J3,"):
                    enviar_peer(MAC_ROBOT_J3, data)
                else:
                    # Comandos globales (STOP, SYNC,...) se envían a ambas placas
                    enviar_peer(MAC_ROBOT_J12, data)
                    enviar_peer(MAC_ROBOT_J3, data)
        elif ch != "\r":
            buffer_usb += ch

    # Robot (ESP-NOW) -> PC (USB)
    mac, msg = e.irecv(0)
    if msg:
        print(msg.decode("utf-8", "ignore"))

    time.sleep_ms(1)