import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool
import cv2
import numpy as np
import time
from collections import deque
from ament_index_python.packages import get_package_share_directory
import os




class UserTrackerNode(Node):
    def __init__(self):
        super().__init__('user_tracker_node')

        # constantes:
        package_path = get_package_share_directory('deteccion_persona_pkg')
        model_path = os.path.join(package_path, 'face_detection_yunet_2026may.onnx')
        camera_index = 0  # en computador: 0->camara laptop, 1->camara externa
        frame_width = 640  # RECALIBRAR RANGO DISTANCIA SI SE CAMBIA
        frame_height = 352 # 352 para que sea divisible por 32 (no influye en ancho de cara)

        # variables
        self.width_avg = 0
        self.no_face_timer = 0
        self.face_detected = False
        self.user_detected = False
        self.faces_widths = list()
        self.near_widths = deque(maxlen=10)  # en vez de list para que no se mueva todo al hacer del list[0]

        # para reducir uso de ram y cpu:
        cv2.setNumThreads(2)  # limita la cantidad de threads a usar (RPI4 tiene 4 cores)
        self.cap = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)  # CAP_V4L2 para iniciar mas rapido
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)
        self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0) # deshabilitar autofocus (deberia mejorar un poco el rendimiento)
        self.cap.set(cv2.CAP_PROP_FPS, 10)  # tal vez cambiar (para evitar frame lag)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1) # solo usa el frame mas nuevo sin "acumular" frames antiguos

        # se lee las dimensiones verdaderas (por si no se actualizaron correctamente los limites)
        w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.detector = cv2.FaceDetectorYN.create(model_path, "", (w, h))

        # este nodo publica la "distancia" (face_width) al topico /user_distance
        self.distance_pub = self.create_publisher(Int32,"/user_distance", 10)
        # tambien publica si esta el usuario o no (face_detected) al topico /user_detected
        self.detected_pub = self.create_publisher(Bool,"/user_detected", 10)
        # se crea un timer para procesar cada 0.1 segundo (10 fps)
        self.timer = self.create_timer(0.1, self.process_frame)
        # tambien se crea un log de este nodo para mandar informacion del proceso
        self.get_logger().info("Inicio de log de camara trasera")

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("No se puede leer camara")
            return
        
        _, faces = self.detector.detect(frame)  # _ = retval (no se usa)
        
        if faces is not None:
            for face in faces:
                box = face[0:4].astype(np.int32)
                self.faces_widths.append(box[2])
        
        nearest_face_width = max(self.faces_widths, default=None)  # detecta a persona mas cercana a camara
        self.faces_widths = list()
        
        if nearest_face_width == None:
            if self.face_detected == True:
                self.no_face_timer = time.monotonic()
                self.near_widths.clear()   # para no mantener valores antiguos
            self.face_detected = False
        else:
            self.face_detected = True
            self.user_detected = True
            self.near_widths.append(nearest_face_width)
        
            # promedio de ancho ultimos 10 frames
            if len(self.near_widths) == 10:
                self.width_avg = int(sum(self.near_widths) / 10)
        
        if not self.face_detected:
            elapsed = time.monotonic() - self.no_face_timer
            if elapsed > 3:
                self.get_logger().warn("No se detecta al usuario")
                self.user_detected = False

        msg1 = Int32()
        msg2 = Bool()
        msg1.data = self.width_avg  # valor objetivo = 35, un rango de 33 a 37 es de ~240cm a ~270cm
        msg2.data = self.user_detected # indica si han pasado 3 segundos desde que no se detecta un usuario
        self.distance_pub.publish(msg1)  # se publica el ancho al topico /user_distance
        self.detected_pub.publish(msg2)

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UserTrackerNode()

    try:
        rclpy.spin(node)  # inicia el nodo y lo mantiene corriendo hasta terminar programa
    except KeyboardInterrupt:  # cambiar por una condicion de termino o solo borrarlo
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()