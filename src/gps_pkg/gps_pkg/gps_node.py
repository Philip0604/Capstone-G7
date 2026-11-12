import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial
from geopy import distance


class GPSNode(Node):
    def __init__(self):
        super().__init__("gps_node")

        # constantes:
        # port especifico para el gps (no cambia):
        port = "/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_7_-_GPS_GNSS_Receiver-if00"
        baud = 9600

        # variables
        self.m_s = 0.0
        self.lat = 0.0
        self.lon = 0.0
        self.old_pos = (0.0, 0.0)
        self.new_pos = (0.0, 0.0)

        self.gps = serial.Serial(port, baud)

        # este nodo publica la distancia recorrida desde la ultima vez al topico /distance_traveled
        self.distance_pub = self.create_publisher(Float32,"/distance_traveled", 10)
        # tambien publica la velocidad aproximada por el gps al topico /speed_aprox
        self.speed_pub = self.create_publisher(Float32,"/speed_aprox", 10)

        # se crea un timer para procesar cada 0.1 segundo (10 fps)
        self.timer = self.create_timer(0.1, self.get_gps_info)  # revisar!!

        # tambien se crea un log de este nodo para mandar informacion del proceso
        self.get_logger().info("Inicio de log de gps")

    def ddm_to_dd(self, lat_lon, coord):  # coord es lat o lon [ddmm.sssss, (direccion)]
        sign = 1
        if coord[1] in ["W", "S"]:
            sign = -1

        if lat_lon == "lat":
            deg = coord[0][0:2]
            minutes = coord[0][2:]
        elif lat_lon == "lon":
            deg = coord[0][0:3]
            minutes = coord[0][3:]

        dd = (float(deg) + float(minutes)/60) * sign
        return dd

    def get_gps_info(self):
        msg = self.gps.readline().decode("utf-8").strip().split(",")
        
        if msg[0] == "$GPVTG":
            if msg[7] != "":
                self.m_s = round(float(msg[7])/3.6, 5) # velocidad aproximada en m/s
        if msg[0] == "$GPGGA":
            if not("" in msg[2:4] or "" in msg[4:6]):
                self.old_pos = (self.lat, self.lon)
                self.lat = self.ddm_to_dd("lat", msg[2:4])
                self.lon = self.ddm_to_dd("lat", msg[4:6])
                self.new_pos = (self.lat, self.lon)
    
        advance = round(distance.distance(self.old_pos, self.new_pos).m, 5)
        
        if msg[0] == "$GPGGA": # para asegurarse que pase 1 vez por segundo (freq de gps)
            msg1 = Float32()
            msg2 = Float32()
            msg1.data = advance
            msg2 = self.m_s
            self.distance_pub.publish(msg1)
            self.speed_pub.publish(msg2)

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GPSNode()

    try:
        rclpy.spin(node)  # inicia el nodo y lo mantiene corriendo hasta terminar programa
    except KeyboardInterrupt:  # cambiar por una condicion de termino o solo borrarlo
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
