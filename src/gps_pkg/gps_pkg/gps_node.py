"""
Nodo GPS (u-blox 7, NMEA por USB).

Publica (1 vez por cada sentencia GGA, ~1 Hz):
  /distance_traveled  (std_msgs/Float32)  metros avanzados desde la GGA anterior
  /speed_aprox        (std_msgs/Float32)  velocidad en m/s (sentencia VTG)

Cambios respecto de la version anterior:
  - Corregido: la longitud se convertia como latitud (2 digitos de grados en vez de 3).
  - Corregido: la primera distancia se medía desde (0, 0) -> miles de km de "avance".
  - Solo usa posiciones con fix valido (campo de calidad de GGA != 0).
  - Lee TODAS las lineas disponibles sin bloquear (antes 1 linea bloqueante por ciclo).
  - Ignora bytes corruptos en vez de caerse con UnicodeDecodeError.
  - Acepta GP/GN (GPGGA, GNGGA, ...).
  - Distancia con haversine (sin geopy: menos RAM y tiempo de arranque).
  - Si la velocidad es menor a 'velocidad_minima' se considera detenido (el ruido
    del GPS detenido sumaba metros falsos).
"""

import math

import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import Float32

RADIO_TIERRA_M = 6371000.0


def haversine_m(lat1, lon1, lat2, lon2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = p2 - p1
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * RADIO_TIERRA_M * math.asin(math.sqrt(a))


def ddm_to_dd(valor, direccion, digitos_grados):
    """'3326.12345','S',2 -> -33.4354 (grados decimales)."""
    grados = float(valor[:digitos_grados])
    minutos = float(valor[digitos_grados:])
    dd = grados + minutos / 60.0
    return -dd if direccion in ("S", "W") else dd


class GPSNode(Node):
    def __init__(self):
        super().__init__("gps_node")

        self.declare_parameter(
            "puerto", "/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_7_-_GPS_GNSS_Receiver-if00")
        self.declare_parameter("baud", 9600)
        self.declare_parameter("velocidad_minima", 0.15)  # m/s

        puerto = self.get_parameter("puerto").value
        baud = int(self.get_parameter("baud").value)
        self.v_min = float(self.get_parameter("velocidad_minima").value)

        self.m_s = 0.0
        self.pos_anterior = None  # (lat, lon) de la ultima GGA valida
        self.buffer = b""

        # timeout=0 -> lecturas no bloqueantes
        self.gps = serial.Serial(puerto, baud, timeout=0)

        self.distance_pub = self.create_publisher(Float32, "/distance_traveled", 10)
        self.speed_pub = self.create_publisher(Float32, "/speed_aprox", 10)

        self.timer = self.create_timer(0.1, self.get_gps_info)
        self.get_logger().info("Inicio de log de gps")

    def get_gps_info(self):
        try:
            n = self.gps.in_waiting
            if n:
                self.buffer += self.gps.read(n)
        except serial.SerialException as e:
            self.get_logger().error(f"Error leyendo GPS: {e}", throttle_duration_sec=5.0)
            return

        # Procesar todas las lineas completas que hayan llegado
        while b"\n" in self.buffer:
            linea, self.buffer = self.buffer.split(b"\n", 1)
            self.procesar_linea(linea.decode("ascii", errors="ignore").strip())

        if len(self.buffer) > 4096:  # basura sin saltos de linea
            self.buffer = b""

    def procesar_linea(self, linea):
        if not linea.startswith("$"):
            return
        campos = linea.split("*")[0].split(",")
        tipo = campos[0][3:]  # 'GGA', 'VTG', ... (sin el prefijo GP/GN)

        try:
            if tipo == "VTG" and len(campos) > 7 and campos[7]:
                self.m_s = round(float(campos[7]) / 3.6, 5)  # km/h -> m/s

            elif tipo == "GGA" and len(campos) > 6:
                avance = 0.0
                fix_valido = campos[6] not in ("", "0") and all(campos[2:6])
                if fix_valido:
                    pos = (ddm_to_dd(campos[2], campos[3], 2), ddm_to_dd(campos[4], campos[5], 3))
                    if self.pos_anterior is not None and self.m_s >= self.v_min:
                        avance = haversine_m(*self.pos_anterior, *pos)
                    self.pos_anterior = pos
                else:
                    self.get_logger().warn("GPS sin fix", throttle_duration_sec=10.0)

                self.distance_pub.publish(Float32(data=float(round(avance, 5))))
                self.speed_pub.publish(Float32(data=float(self.m_s)))
        except ValueError:
            pass  # sentencia corrupta: se ignora

    def destroy_node(self):
        try:
            self.gps.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GPSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
