#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import serial


class SerialBridge(Node):
    def __init__(self):
        super().__init__("serial_bridge")

        # Ajustar el puerto al que corresponda a la Arduino.
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)

        port = self.get_parameter("port").value
        baudrate = self.get_parameter("baudrate").value

        self.serial_port = serial.Serial(port=port, baudrate=baudrate, timeout=0.1, write_timeout=0.1)

        # La placa puede reiniciarse al abrir el puerto USB.
        self.last_message_time = self.get_clock().now()
        self.subscription = self.create_subscription(Vector3, "/line_errors", self.error_callback, 10)

        # Si no llegan mensajes recientes, enviar orden de parada.
        self.watchdog = self.create_timer(0.1, self.check_timeout)

        self.get_logger().info(f"Puente serial activo: {port} a {baudrate} baudios")

    def error_callback(self, msg):
        lateral = float(msg.x)
        angular = float(msg.y)

        if not (-1.0 <= lateral <= 1.0 and -1.0 <= angular <= 1.0):
            self.get_logger().warning("Error fuera de rango")
            return

        linea = f"{lateral:.4f},{angular:.4f}\n"
        self.serial_port.write(linea.encode("ascii"))
        self.last_message_time = self.get_clock().now()

    def check_timeout(self):
        elapsed = (self.get_clock().now() - self.last_message_time).nanoseconds / 1e9

        if elapsed > 0.5:
            self.serial_port.write(b"STOP\n")

    def destroy_node(self):
        if hasattr(self, "serial_port") and self.serial_port.is_open:
            self.serial_port.write(b"STOP\n")
            self.serial_port.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = SerialBridge()
        rclpy.spin(node)
    except (serial.SerialException, OSError) as exc:
        print(f"Error de comunicacion serial: {exc}")
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()