import rclpy
from rclpy.node import Node
import pyrealsense2 as rs
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraNode(Node):
    def __init__(self):
        super().__init__("camera_node")

        # Publicadores
        self.color_publisher = self.create_publisher(Image, "/camera/color/image", 10)
        self.depth_publisher = self.create_publisher(Image, "/camera/depth/image", 10)
        
        self.bridge = CvBridge() # Conversor OpenCV <-> ROS

        # Configuración de RealSense
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 6)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 6)
        self.pipeline.start(config)

        # Ejecutar aproximadamente cada 1/15 segundos
        self.timer = self.create_timer(1.0 / 15.0, self.publicar_frames)

        self.get_logger().info("Nodo de cámara iniciado")

    def publicar_frames(self):
        # Obtener frames de la RealSense
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if not color_frame or not depth_frame:
            self.get_logger().warning("No se recibieron frames válidos")
            return

        # Convertir a NumPy
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        self.get_logger().info(f"Color image: shape={color_image.shape}, dtype={color_image.dtype}")
        self.get_logger().info(f"Depth image: shape={depth_image.shape}, dtype={depth_image.dtype}")

        # Convertir a mensajes ROS
        color_msg = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
        depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding="16UC1")

        # Publicar
        self.color_publisher.publish(color_msg)
        self.depth_publisher.publish(depth_msg)

    def destroy_node(self):
        self.pipeline.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()