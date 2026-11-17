# Nodo de la camara RealSense D435.
# Publica:
#   /camera/color/image   (sensor_msgs/Image, bgr8)
#   /camera/depth/image   (sensor_msgs/Image, 16UC1, en milimetros) -> solo si publicar_profundidad:=true

import cv2
import numpy as np
import pyrealsense2 as rs
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class CameraNode(Node):
    def __init__(self):
        super().__init__("realsense_node")

        self.declare_parameter("ancho", 640)
        self.declare_parameter("alto", 480)
        self.declare_parameter("fps", 15)
        self.declare_parameter("ancho_publicado", 320)
        self.declare_parameter("publicar_profundidad", False)

        ancho = int(self.get_parameter("ancho").value)
        alto = int(self.get_parameter("alto").value)
        fps = int(self.get_parameter("fps").value)
        anch_pub = int(self.get_parameter("ancho_publicado").value)
        self.tam_pub = None
        if 0 < anch_pub < ancho:
            self.tam_pub = (anch_pub, int(round(alto * anch_pub / ancho)))
        self.con_profundidad = bool(self.get_parameter("publicar_profundidad").value)

        self.color_publisher = self.create_publisher(Image, "/camera/color/image", qos_profile_sensor_data)
        self.depth_publisher = None
        if self.con_profundidad:
            self.depth_publisher = self.create_publisher(Image, "/camera/depth/image", qos_profile_sensor_data)

        # Configuracion de RealSense
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, ancho, alto, rs.format.bgr8, fps)
        if self.con_profundidad:
            config.enable_stream(rs.stream.depth, ancho, alto, rs.format.z16, fps)
        self.pipeline.start(config)

        # Se revisa el doble de rapido que los fps; poll_for_frames no bloquea.
        self.timer = self.create_timer(1.0 / (2.0 * fps), self.publicar_frames)
        self.ultimo_color = -1

        self.get_logger().info(
            f"Nodo de camara iniciado: {ancho}x{alto} @ {fps} fps, profundidad={'si' if self.con_profundidad else 'no'}")

    def publicar_frames(self):
        frames = self.pipeline.poll_for_frames()
        if not frames:
            return
        color_frame = frames.get_color_frame()
        if not color_frame:
            return
        n = color_frame.get_frame_number()
        if n == self.ultimo_color:
            return
        self.ultimo_color = n

        stamp = self.get_clock().now().to_msg()

        color_image = np.asanyarray(color_frame.get_data())
        if self.tam_pub is not None:
            color_image = cv2.resize(color_image, self.tam_pub, interpolation=cv2.INTER_AREA)
        color_msg = cv2_to_imgmsg(color_image, encoding="bgr8")
        color_msg.header.stamp = stamp
        color_msg.header.frame_id = "camera_color_optical_frame"
        self.color_publisher.publish(color_msg)

        # Profundidad solo si esta activada y alguien la esta escuchando
        if self.depth_publisher is not None and self.depth_publisher.get_subscription_count() > 0:
            depth_frame = frames.get_depth_frame()
            if depth_frame:
                depth_msg = cv2_to_imgmsg_16uc1(np.asanyarray(depth_frame.get_data()))
                depth_msg.header.stamp = stamp
                depth_msg.header.frame_id = "camera_depth_optical_frame"
                self.depth_publisher.publish(depth_msg)

    def destroy_node(self):
        try:
            self.pipeline.stop()
        except RuntimeError:
            pass
        super().destroy_node()


def cv2_to_imgmsg(cv_img: np.ndarray, encoding: str = "bgr8") -> Image:
    cv_img = np.ascontiguousarray(cv_img)
    msg = Image()
    msg.height, msg.width = cv_img.shape[:2]
    msg.encoding = encoding
    msg.is_bigendian = 0
    msg.step = cv_img.strides[0]
    msg.data = cv_img.tobytes()
    return msg


def cv2_to_imgmsg_16uc1(cv_img: np.ndarray) -> Image:
    if cv_img.dtype != np.uint16:
        cv_img = cv_img.astype(np.uint16)
    cv_img = np.ascontiguousarray(cv_img)
    msg = Image()
    msg.height, msg.width = cv_img.shape[:2]
    msg.encoding = "16UC1"
    msg.is_bigendian = 0
    msg.step = cv_img.strides[0]
    msg.data = cv_img.tobytes()
    return msg


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
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