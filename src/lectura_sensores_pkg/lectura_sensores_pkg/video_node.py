"""
Nodo de PRUEBA: publica un video como si fuera la camara RealSense.

Publica:
  /camera/color/image   (sensor_msgs/Image, bgr8)  mismo topico, QoS y formato que realsense_node

Asi el resto de los nodos (deteccion de pista, pid_message_node, ...) funcionan
igual que con la camara real, sin necesitar el robot.

Uso:
  ros2 run lectura_sensores_pkg video_node --ros-args -p video:=videos/input/Pista_zigzag.mp4
  ros2 launch lectura_sensores_pkg video_prueba.launch.py video:=videos/input/Pista_curva.mp4

Parametros:
  video            (str)   ruta al .mp4 (absoluta o relativa a la carpeta desde donde se lanza)
  fps              (float) 15.0   igual que la RealSense
  repetir          (bool)  True   vuelve al inicio cuando termina el video
  ancho, alto      (int)   640x480  se re-escala a la resolucion de la RealSense
  ancho_publicado  (int)   320    igual que realsense_node (0 = sin reducir)
"""

import os

import cv2
import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class VideoNode(Node):
    def __init__(self):
        super().__init__("video_node")

        self.declare_parameter("video", "videos/input/Pista_zigzag.mp4")
        # dynamic_typing: acepta fps:=15 y fps:=15.0
        self.declare_parameter("fps", 15.0, ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter("repetir", True)
        self.declare_parameter("ancho", 640)
        self.declare_parameter("alto", 480)
        self.declare_parameter("ancho_publicado", 320)

        ruta = os.path.expanduser(str(self.get_parameter("video").value))
        fps = float(self.get_parameter("fps").value)
        self.repetir = bool(self.get_parameter("repetir").value)
        ancho = int(self.get_parameter("ancho").value)
        alto = int(self.get_parameter("alto").value)
        anch_pub = int(self.get_parameter("ancho_publicado").value)

        # Mismo procesamiento que realsense_node: captura a ancho x alto y
        # publica reducido a ancho_publicado (manteniendo la proporcion).
        self.tam_camara = (ancho, alto)
        if 0 < anch_pub < ancho:
            self.tam_pub = (anch_pub, int(round(alto * anch_pub / ancho)))
        else:
            self.tam_pub = self.tam_camara

        if not os.path.isfile(ruta):
            raise FileNotFoundError(
                f"No existe el video '{ruta}' (carpeta actual: {os.getcwd()}). "
                "Usa una ruta absoluta o lanza desde la raiz del proyecto.")
        self.cap = cv2.VideoCapture(ruta)
        if not self.cap.isOpened():
            raise RuntimeError(f"OpenCV no pudo abrir '{ruta}'")
        self.ruta = ruta

        self.publisher = self.create_publisher(Image, "/camera/color/image", qos_profile_sensor_data)
        self.timer = self.create_timer(1.0 / fps, self.publicar_frame)

        total = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
        self.get_logger().info(
            f"Publicando '{os.path.basename(ruta)}' ({total} frames) a {fps:.0f} fps, "
            f"{self.tam_pub[0]}x{self.tam_pub[1]} en /camera/color/image")

    def publicar_frame(self):
        ok, frame = self.cap.read()
        if not ok:
            if not self.repetir:
                self.get_logger().info("Fin del video", once=True)
                return
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ok, frame = self.cap.read()
            if not ok:
                return
            self.get_logger().info("Video reiniciado")

        frame = cv2.resize(frame, self.tam_camara, interpolation=cv2.INTER_AREA)
        if self.tam_pub != self.tam_camara:
            frame = cv2.resize(frame, self.tam_pub, interpolation=cv2.INTER_AREA)

        frame = np.ascontiguousarray(frame)
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_color_optical_frame"
        msg.height, msg.width = frame.shape[:2]
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = frame.strides[0]
        msg.data = frame.tobytes()
        self.publisher.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = VideoNode()
    except (FileNotFoundError, RuntimeError) as e:
        print(f"[video_node] {e}")
        rclpy.shutdown()
        return
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
