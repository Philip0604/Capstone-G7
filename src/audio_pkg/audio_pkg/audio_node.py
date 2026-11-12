import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32


class MainNode(Node):
    def __init__(self):
        super().__init__('audio_node')

        # constantes:


        # variables:
        self.recent_distance = 0.0
        self.speed_aprox = 0.0
        self.user_found = False
        self.total_distance = 0

        # subscripciones y timers:
        self.sub_distance = self.create_subscription(Float32, "/distance_traveled", self.distance_callback, 10)
        self.sub_speed = self.create_subscription(Float32, "/speed_aprox", self.speed_callback, 10)
        self.sub_user_detect = self.create_subscription(Bool, "/user_detected", self.user_detect_callback, 10)
        #self.sub_obstacle = self.create_subscription(Float32, "_______", self._____, 10)

        self.timer = self.create_timer(0.1, self.arduino_info)   # audio, velocidad, etc (revisar)
        self.get_logger().info("Inicio de log nodo principal")

    def distance_callback(self, msg):
        self.recent_distance = msg.data
        self.total_distance += self.recent_distance

    def speed_callback(self, msg):
        self.speed_aprox = msg.data

    def user_detect_callback(self, msg):
        self.user_found = msg.data

    def arduino_info(self):
        self.get_logger().info(f"Distancia recorrida: {self.total_distance:.2f} m")
        self.get_logger().info(f"Velocidad aproximada: {self.speed_aprox:.2f} m/s")
        self.get_logger().info(f"Usuario encontrado: {'Sí' if self.user_found else 'No'}")

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MainNode()

    try:
        rclpy.spin(node)  # inicia el nodo y lo mantiene corriendo hasta terminar programa
    except KeyboardInterrupt:  # cambiar por una condicion de termino o solo borrarlo
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()