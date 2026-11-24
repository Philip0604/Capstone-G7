import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int32, Float32
from ament_index_python.packages import get_package_share_directory


class PIDMessageNode(Node):
    def __init__(self):
        super().__init__("pid_message_node")

        # constantes:
        self.declare_parameter("frecuencia", 15.0)     # Hz
        self.declare_parameter("timeout_pista", 0.3)   # s
        self.declare_parameter("timeout_distance", 0.3)# s

        frecuencia = float(self.get_parameter("frecuencia").value)
        self.timeout_pista = float(self.get_parameter("timeout_pista").value)
        self.timeout_distance = float(self.get_parameter("timeout_distance").value)

        # variables: None significa que no se recibio el valor aun, y no se publica nada hasta que se reciban los 3 valores
        self.distance = None  # recibe un int
        self.lat_error = None
        self.ang_error = None
        self.t_lat = 0.0
        self.t_ang = 0.0
        self.t_distance = 0.0

        # recibe los diferentes valores que vienen de diferentes nodos
        self.sub_distance = self.create_subscription(Int32, "/user_distance", self.user_distance_callback, 10)
        self.sub_lat_error = self.create_subscription(Float32, "/lateral_error", self.lat_error_callback, 10)
        self.sub_ang_error = self.create_subscription(Float32, "/angular_error", self.ang_error_callback, 10)

        # este nodo publica las variables del pid al topico /pid_data
        self.pid_pub = self.create_publisher(Vector3,"/pid_data", 10)

        # recibe los valores constantemente y los envia con 15fps (mismo fps que camara con mayor fps)
        self.timer = self.create_timer(1.0 / frecuencia, self.public_values)
        
        # tambien se crea un log de este nodo para mandar informacion del proceso
        #self.get_logger().info("Inicio de log de los mensajes de valores para pid")
    
    def lat_error_callback(self, msg):
        self.lat_error = msg.data
        self.t_lat = time.monotonic()
        self.get_logger().info(f"Error lateral: {self.lat_error}", throttle_duration_sec=2.0)

    def ang_error_callback(self, msg):
        self.ang_error = msg.data
        self.t_ang = time.monotonic()
        self.get_logger().info(f"Error angular: {self.ang_error}", throttle_duration_sec=2.0)

    def user_distance_callback(self, msg):
        self.distance = msg.data
        self.t_distance = time.monotonic()
        self.get_logger().info(f"Distancia al usuario: {self.distance} (ancho px cara)", throttle_duration_sec=2.0)

    def public_values(self):
        # if self.lat_error is None or self.ang_error is None or self.distance is None:
        #     return
        ahora = time.monotonic()
        # if ahora - self.t_lat > self.timeout_pista or ahora - self.t_ang > self.timeout_pista:
        #     self.get_logger().warn("Pista perdida: no se envian ordenes al Arduino", throttle_duration_sec=2.0)
        #     return
        # if ahora - self.t_distance > self.timeout_distance:
        #     self.get_logger().warn("Distancia al usuario perdida: no se envian ordenes al Arduino", throttle_duration_sec=2.0)
        #     return
        if self.lat_error is None:
            self.get_logger().warn("Error lateral no recibido: no se envian ordenes al Arduino", throttle_duration_sec=2.0)
            self.lat_error = 0.5
        if self.ang_error is None:
            self.get_logger().warn("Error angular no recibido: no se envian ordenes al Arduino", throttle_duration_sec=2.0)
            self.ang_error = 0.5
        if self.distance is None:
            self.get_logger().warn("Distancia al usuario no recibida: no se envian ordenes al Arduino", throttle_duration_sec=2.0)
            self.distance = 0.5
        # solo se publica un vector cuando llegaron los 3 valores
        msg_vector = Vector3()
        msg_vector.x = float(self.lat_error)
        msg_vector.y = float(self.ang_error)
        msg_vector.z = float(self.distance)
        self.pid_pub.publish(msg_vector)

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PIDMessageNode()

    try:
        rclpy.spin(node)  # inicia el nodo y lo mantiene corriendo hasta terminar programa
    except KeyboardInterrupt:  # cambiar por una condicion de termino o solo borrarlo
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()