import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int32, Float32
from ament_index_python.packages import get_package_share_directory


class PIDMessageNode(Node):
    def __init__(self):
        super().__init__("pid_message_node")

        # constantes:
        package_path = get_package_share_directory()  # falta poner locacion package

        # variables:
        self.distance = 0  # recibe un int
        self.lat_error = 0.0
        self.ang_error = 0.0

        # recibe los diferentes valores que vienen de diferentes nodos
        self.sub_distance = self.create_subscription(Int32, "/user_distance", self.user_distance_callback, 10)
        self.sub_lat_error = self.create_subscription(Float32, "/lateral_error", self.lat_error_callback, 10)
        self.sub_ang_error = self.create_subscription(Float32, "/angular_error", self.ang_error_callback, 10)

        # este nodo publica las variables del pid al topico /pid_data
        self.pid_pub = self.create_publisher(Vector3,"/pid_data", 10)

        # recibe los valores constantemente y los envia con 15fps (mismo fps que camara con mayor fps)
        self.timer = self.create_timer(1.0 / 15.0, self.public_values)
        
        # tambien se crea un log de este nodo para mandar informacion del proceso
        #self.get_logger().info("Inicio de log de los mensajes de valores para pid")
    
    def lat_error_callback(self, msg):
        self.lat_error = msg.data

    def ang_error_callback(self, msg):
        self.ang_error = msg.data

    def user_distance_callback(self, msg):
        self.distance = msg.data

    def public_values(self):
        # solo se publica un vector cuando llegaron los 3 valores
        if self.x is not None and self.y is not None and self.z is not None:
            msg_vector = Vector3()
            msg_vector.x = float(self.x)
            msg_vector.y = float(self.y)
            msg_vector.z = float(self.z)
            self.pid_pub.publish(msg_vector)

    def destroy_node(self):
        self.cap.release()
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