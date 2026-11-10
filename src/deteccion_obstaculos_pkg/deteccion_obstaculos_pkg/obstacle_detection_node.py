
import rclpy
from rclpy.node import Node


class DeteccionObstaculosNode(Node):

    def __init__(self):
        super().__init__('deteccion_obstaculos_node')

        # Parámetros
        self.frecuencia = 10.0  # Hz

        # Suscriptores
        # TODO: Suscribirse al tópico de profundidad de la RealSense.

        # Publicadores
        # TODO: Publicar información de obstáculos detectados.

        # Temporizador
        self.timer = self.create_timer(
            1.0 / self.frecuencia,
            self.procesar_frame
        )

        self.get_logger().info(
            'Nodo de detección de obstáculos iniciado.'
        )

    def procesar_frame(self):
        """
        Ejecuta periódicamente el procesamiento de los datos
        recibidos desde la cámara.
        """

        # TODO: Obtener frame de profundidad.
        # TODO: Detectar obstáculos.
        # TODO: Publicar resultados.

        pass

    def detectar_obstaculos(self, frame_profundidad):
        """
        Analiza un frame de profundidad y detecta obstáculos.

        Args:
            frame_profundidad: Imagen de profundidad de la RealSense.

        Returns:
            Información de los obstáculos detectados.
        """

        # TODO: Implementar algoritmo de detección.

        pass


def main(args=None):
    rclpy.init(args=args)

    node = DeteccionObstaculosNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
