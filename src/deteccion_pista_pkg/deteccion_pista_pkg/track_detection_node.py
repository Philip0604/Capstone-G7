# Nodo de deteccion de pista.

# Suscribe:
#   /camera/color/image   (sensor_msgs/Image, bgr8)
# Publica:
#   /lateral_error        (std_msgs/Float32)  [-1, 1]  0 = centro entre las 2 lineas
#   /angular_error        (std_msgs/Float32)  [-1, 1]  0 = lineas simetricas ("/" y "\")
#   /pista/detectada      (std_msgs/Bool)     True si se encontro un par valido
#   Los errores SOLO se publican cuando hay un par valido. Asi pid_message_node
#   deja de mandar ordenes y el Arduino detiene el robot por timeout si se pierde la pista.

# Signos (ver deteccion.py): error > 0 => el robot debe girar a la izquierda.

# Parametros ROS (ros2 run ... --ros-args -p mostrar_ventana:=false):
#   topico_color        (str)   '/camera/color/image'
#   mostrar_ventana     (bool)  True   ventana con las lineas detectadas
#   vista_cada_n        (int)   2      refresca la ventana cada N frames (ahorra CPU)
#   filtro_alfa         (float) 0.5    filtro exponencial de los errores (1.0 = sin filtro)

import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32

from deteccion_pista_pkg.deteccion import DetectorPista

VENTANA = "Pista - lineas detectadas"


class LaneDetectionNode(Node):
    def __init__(self):
        super().__init__("track_detection_node")

        self.declare_parameter("topico_color", "/camera/color/image")
        self.declare_parameter("mostrar_ventana", True)
        self.declare_parameter("vista_cada_n", 2)
        self.declare_parameter("filtro_alfa", 0.5)

        topico = self.get_parameter("topico_color").value
        self.mostrar = bool(self.get_parameter("mostrar_ventana").value)
        self.vista_cada_n = max(1, int(self.get_parameter("vista_cada_n").value))
        self.alfa = float(self.get_parameter("filtro_alfa").value)

        # Sin pantalla (ej: por SSH sin -X) imshow cae: se desactiva solo.
        if self.mostrar and not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
            self.get_logger().warn("No hay DISPLAY: se desactiva la ventana")
            self.mostrar = False

        cv2.setNumThreads(2)  # dejar nucleos libres para los otros nodos

        # QoS "sensor": best effort y cola de 1 -> si la Raspberry se atrasa se
        # descartan frames viejos en vez de acumular retraso.
        self.sub = self.create_subscription(Image, topico, self.image_callback, qos_profile_sensor_data)

        self.pub_lat = self.create_publisher(Float32, "/lateral_error", 10)
        self.pub_ang = self.create_publisher(Float32, "/angular_error", 10)
        self.pub_ok = self.create_publisher(Bool, "/pista/detectada", 10)

        self.detector = None
        self.lat_f = None
        self.ang_f = None

        # Estadisticas
        self.n_frames = 0
        self.n_ok = 0
        self.t_proc = 0.0
        self.t_stats = time.monotonic()

        self.get_logger().info(f"Nodo de deteccion de pista iniciado (escuchando {topico})")

    def image_callback(self, msg):
        frame = imgmsg_to_cv2(msg)
        if frame is None:
            return
        h, w = frame.shape[:2]
        if self.detector is None or self.detector.ancho_in != w or self.detector.alto_in != h:
            self.detector = DetectorPista(w, h)
            self.get_logger().info(f"Imagen de entrada {w}x{h} -> se procesa a {self.detector.w}x{self.detector.h}")

        self.n_frames += 1
        dibujar = self.mostrar and (self.n_frames % self.vista_cada_n == 0)

        t0 = time.perf_counter()
        resultado, salida = self.detector.procesar(frame, dibujar=dibujar)
        self.t_proc += time.perf_counter() - t0

        hay_par = resultado["hay_pareja"]
        self.pub_ok.publish(Bool(data=hay_par))

        if hay_par:
            self.n_ok += 1
            lat, ang = resultado["error_lateral"], resultado["error_angular"]
            if self.lat_f is None:
                self.lat_f, self.ang_f = lat, ang
            else:
                self.lat_f = self.alfa * lat + (1.0 - self.alfa) * self.lat_f
                self.ang_f = self.alfa * ang + (1.0 - self.alfa) * self.ang_f
            self.pub_lat.publish(Float32(data=float(self.lat_f)))
            self.pub_ang.publish(Float32(data=float(self.ang_f)))
        else:
            self.lat_f = self.ang_f = None  # al recuperar la pista se parte de cero

        # if dibujar and salida is not None:
        #     cv2.imshow(VENTANA, salida)
        #     cv2.waitKey(1)

        self.reportar()

    def reportar(self):
        ahora = time.monotonic()
        dt = ahora - self.t_stats
        if dt < 5.0:
            return
        if self.n_frames:
            self.get_logger().info(
                f"{self.n_frames / dt:.1f} fps | par detectado {100.0 * self.n_ok / self.n_frames:.0f}% | "
                f"{1000.0 * self.t_proc / self.n_frames:.1f} ms/frame")
        else:
            self.get_logger().warn("No llegan imagenes de la camara")
        self.n_frames = self.n_ok = 0
        self.t_proc = 0.0
        self.t_stats = ahora

    def destroy_node(self):
        if self.mostrar:
            cv2.destroyAllWindows()
        super().destroy_node()


def imgmsg_to_cv2(msg: Image):
    """Image (bgr8/rgb8) -> np.ndarray BGR sin copiar (respeta el padding de 'step')."""
    if msg.encoding not in ("bgr8", "rgb8"):
        return None
    img = np.ndarray(shape=(msg.height, msg.width, 3), dtype=np.uint8,
                     buffer=msg.data, strides=(msg.step, 3, 1))
    if msg.encoding == "rgb8":
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    return img


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectionNode()
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