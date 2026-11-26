"""
Nodo de deteccion de obstaculos (RealSense D435, profundidad).

Suscribe:
  /camera/depth/image        (sensor_msgs/Image, 16UC1 en mm)  -> realsense_node con
                                                                 publicar_profundidad:=true
  /camera/depth/camera_info  (sensor_msgs/CameraInfo)          intrinsecos reales (opcional)
Publica:
  /obstaculo/detectado       (std_msgs/Bool)      hay un obstaculo a <= distancia_max_m
  /obstaculo/distancia       (std_msgs/Float32)   m (horizontal) al mas cercano, -1.0 si no hay
  /obstaculo/posicion        (geometry_msgs/Vector3) x = adelante, y = lateral (+ derecha),
                                                  z = altura sobre el piso (m). Solo si hay.
  /obstaculo/mascara         (sensor_msgs/Image, mono8) solo si publicar_mascara:=true
Servicio:
  /obstaculos/calibrar       (std_srvs/Trigger)   calibra el piso con los proximos frames

CALIBRACION (hacer una vez, y de nuevo si se mueve la camara):
  1. Robot en piso plano, SIN objetos ~4 m delante de la camara.
  2. ros2 service call /obstaculos/calibrar std_srvs/srv/Trigger
     (o lanzar con -p calibrar_al_iniciar:=true)
  3. Revisar el log: "Calibracion OK: altura=0.25x m, inclinacion=..."
     Se guarda en ruta_calibracion y se carga sola en los siguientes arranques.
  Sin archivo de calibracion se usan altura_camara_m e inclinacion_deg (manual).
"""

import os
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, Float32
from std_srvs.srv import Trigger

from deteccion_obstaculos_pkg.obstaculos import (Calibracion, Calibrador, DetectorObstaculos,
                                                 ErrorCalibracion, Intrinsecos)


class ObstacleDetectionNode(Node):

    def __init__(self):
        super().__init__('obstacle_detection_node')

        dp = self.declare_parameter
        dp('topico_profundidad', '/camera/depth/image')
        dp('topico_info', '/camera/depth/camera_info')
        dp('frecuencia_max', 10.0)          # Hz (0 = procesar todos los frames)
        # Deteccion
        dp('distancia_max_m', 5.0)
        dp('distancia_min_m', 0.3)          # la D435 no mide bien mas cerca
        dp('altura_min_m', 0.10)            # mas bajo que esto se considera piso
        dp('altura_max_m', 1.8)             # mas alto: ramas, techos (no chocan)
        dp('tolerancia_por_metro', 0.02)    # +2 cm de altura_min por cada metro
        dp('ancho_corredor_m', 0.0)         # 0 = todo el campo de vision
        dp('paso', 4)                       # se usa 1 de cada 'paso' pixeles
        dp('min_celdas', 8)                 # tamano minimo de un obstaculo (celdas)
        dp('frames_confirmar', 2)           # frames seguidos para declarar obstaculo
        dp('frames_liberar', 3)             # frames seguidos sin obstaculo para liberarlo
        # Calibracion
        dp('altura_camara_m', 0.25)
        dp('inclinacion_deg', 0.0)          # > 0: camara mirando hacia abajo
        dp('ruta_calibracion', '~/.ros/calibracion_obstaculos.json')
        dp('calibrar_al_iniciar', False)
        dp('frames_calibracion', 15)
        dp('tolerancia_altura_m', 0.15)     # 0 = no validar contra altura_camara_m
        # Intrinsecos por defecto (si no llega camera_info), para 640 px de ancho
        dp('fx_640', Intrinsecos.FX_640)
        dp('fy_640', Intrinsecos.FY_640)
        dp('publicar_mascara', False)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.ruta_cal = os.path.expanduser(str(g('ruta_calibracion')))
        self.periodo_min = 1.0 / g('frecuencia_max') if g('frecuencia_max') > 0 else 0.0
        self.n_confirmar = max(1, int(g('frames_confirmar')))
        self.n_liberar = max(1, int(g('frames_liberar')))
        self.fx_640, self.fy_640 = float(g('fx_640')), float(g('fy_640'))
        self.altura_esperada = float(g('altura_camara_m'))
        self.tol_altura = float(g('tolerancia_altura_m'))
        self.n_frames_cal = int(g('frames_calibracion'))
        self.publicar_mascara = bool(g('publicar_mascara'))

        self.detector = DetectorObstaculos(
            self.cargar_calibracion(),
            distancia_max=float(g('distancia_max_m')), distancia_min=float(g('distancia_min_m')),
            altura_min=float(g('altura_min_m')), altura_max=float(g('altura_max_m')),
            tolerancia_por_metro=float(g('tolerancia_por_metro')),
            ancho_corredor=float(g('ancho_corredor_m')), paso=int(g('paso')),
            min_celdas=int(g('min_celdas')))

        self.intr = None            # de camera_info
        self.calibrador = None
        self.t_ultimo = 0.0
        self.confirmado = False
        self.cuenta_si = self.cuenta_no = 0
        self.ultima_distancia = -1.0

        self.create_subscription(Image, g('topico_profundidad'), self.depth_callback,
                                 qos_profile_sensor_data)
        self.create_subscription(CameraInfo, g('topico_info'), self.info_callback,
                                 qos_profile_sensor_data)
        self.pub_det = self.create_publisher(Bool, '/obstaculo/detectado', 10)
        self.pub_dist = self.create_publisher(Float32, '/obstaculo/distancia', 10)
        self.pub_pos = self.create_publisher(Vector3, '/obstaculo/posicion', 10)
        self.pub_mask = self.create_publisher(Image, '/obstaculo/mascara', 1)
        self.create_service(Trigger, '/obstaculos/calibrar', self.calibrar_callback)

        if g('calibrar_al_iniciar'):
            self.iniciar_calibracion()

        self.get_logger().info(
            f'Deteccion de obstaculos iniciada (hasta {self.detector.distancia_max:.1f} m, '
            f'altura > {self.detector.altura_min:.2f} m). {self.detector.calibracion}')

    # ------------------------------------------------------------ calibracion
    def cargar_calibracion(self):
        if os.path.isfile(self.ruta_cal):
            try:
                cal = Calibracion.cargar(self.ruta_cal)
                self.get_logger().info(f'Calibracion cargada de {self.ruta_cal}')
                return cal
            except (OSError, ValueError, KeyError) as e:
                self.get_logger().error(f'No se pudo leer {self.ruta_cal}: {e}')
        cal = Calibracion.desde_angulos(float(self.get_parameter('altura_camara_m').value),
                                        float(self.get_parameter('inclinacion_deg').value))
        self.get_logger().warn(
            'Sin calibracion guardada: se usa altura/inclinacion manual. Para calibrar: '
            'ros2 service call /obstaculos/calibrar std_srvs/srv/Trigger')
        return cal

    def iniciar_calibracion(self):
        self.calibrador = Calibrador(self.n_frames_cal, altura_esperada=self.altura_esperada,
                                     tolerancia_altura=self.tol_altura)
        self.get_logger().info(
            f'Calibrando piso con {self.n_frames_cal} frames: mantener despejado delante del robot')

    def calibrar_callback(self, request, response):
        if self.calibrador is not None:
            response.success = False
            response.message = 'Ya hay una calibracion en curso'
            return response
        self.iniciar_calibracion()
        response.success = True
        response.message = (f'Calibracion iniciada ({self.n_frames_cal} frames). '
                            'Ver el log del nodo para el resultado.')
        return response

    def paso_calibracion(self, prof, intr):
        try:
            cal = self.calibrador.agregar(prof, intr)
        except ErrorCalibracion as e:
            self.calibrador = None
            self.get_logger().error(f'{e}. Se mantiene: {self.detector.calibracion}')
            return
        if cal is None:
            return
        self.calibrador = None
        self.detector.calibracion = cal
        try:
            cal.guardar(self.ruta_cal)
            donde = f'guardada en {self.ruta_cal}'
        except OSError as e:
            donde = f'NO se pudo guardar ({e})'
        self.get_logger().info(
            f'Calibracion OK: altura={cal.altura:.3f} m, inclinacion={cal.inclinacion_deg:+.2f} deg, '
            f'roll={cal.roll_deg:+.2f} deg ({donde})')

    # ---------------------------------------------------------------- entradas
    def info_callback(self, msg):
        k = msg.k
        if k[0] > 0 and k[4] > 0:
            self.intr = Intrinsecos(k[0], k[4], k[2], k[5], msg.width, msg.height)

    def intrinsecos(self, ancho, alto):
        if self.intr is not None:
            return self.intr.escalar_a(ancho, alto)
        return Intrinsecos.por_defecto(ancho, alto, self.fx_640, self.fy_640)

    def depth_callback(self, msg):
        prof = imgmsg_a_profundidad(msg)
        if prof is None:
            self.get_logger().warn(f'Encoding de profundidad no soportado: {msg.encoding}',
                                   throttle_duration_sec=10.0)
            return
        intr = self.intrinsecos(msg.width, msg.height)

        if self.calibrador is not None:
            self.paso_calibracion(prof, intr)

        ahora = time.monotonic()
        if self.periodo_min and ahora - self.t_ultimo < self.periodo_min:
            return
        self.t_ultimo = ahora

        res = self.detector.procesar(prof, intr)
        self.publicar(res, msg.header)

    # ---------------------------------------------------------------- salidas
    def publicar(self, res, header):
        # Histeresis: evita que la salida "parpadee" con ruido de un frame
        if res['hay_obstaculo']:
            self.cuenta_si, self.cuenta_no = self.cuenta_si + 1, 0
            self.ultima_distancia = res['distancia']
        else:
            self.cuenta_si, self.cuenta_no = 0, self.cuenta_no + 1
        if not self.confirmado and self.cuenta_si >= self.n_confirmar:
            self.confirmado = True
            self.get_logger().info(f"Obstaculo a {res['distancia']:.2f} m "
                                   f"(lateral {res['lateral']:+.2f} m, alto {res['altura']:.2f} m)")
        elif self.confirmado and self.cuenta_no >= self.n_liberar:
            self.confirmado = False
            self.get_logger().info('Camino despejado')

        self.pub_det.publish(Bool(data=self.confirmado))
        self.pub_dist.publish(Float32(data=float(self.ultima_distancia if self.confirmado else -1.0)))
        if self.confirmado and res['hay_obstaculo']:
            self.pub_pos.publish(Vector3(x=res['adelante'], y=res['lateral'], z=res['altura']))

        if self.publicar_mascara and self.pub_mask.get_subscription_count() > 0:
            m = np.ascontiguousarray(res['mascara'] * 255)
            img = Image()
            img.header = header
            img.height, img.width = m.shape
            img.encoding, img.step, img.data = 'mono8', m.shape[1], m.tobytes()
            self.pub_mask.publish(img)


def imgmsg_a_profundidad(msg):
    """Image 16UC1/mono16 (mm) o 32FC1 (m) -> np.ndarray uint16 en mm (respeta 'step')."""
    if msg.encoding in ('16UC1', 'mono16'):
        return np.ndarray((msg.height, msg.width), np.uint16, buffer=msg.data,
                          strides=(msg.step, 2))
    if msg.encoding == '32FC1':
        m = np.ndarray((msg.height, msg.width), np.float32, buffer=msg.data, strides=(msg.step, 4))
        return np.nan_to_num(m * 1000.0, nan=0.0, posinf=0.0).clip(0, 65535).astype(np.uint16)
    return None


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
