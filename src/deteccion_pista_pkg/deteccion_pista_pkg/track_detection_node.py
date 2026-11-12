import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
import message_filters
# from cv_bridge import CvBridge
import cv2
import numpy as np
from deteccion_pista_pkg import parametros as p

class LaneDetectionNode(Node):
    def __init__(self):
        super().__init__("lane_detection_node")

        # self.bridge = CvBridge()
        self.color_sub = message_filters.Subscriber(self, Image, "/camera/color/image")
        self.depth_sub = message_filters.Subscriber(self, Image, "/camera/depth/image")
        self.get_logger().info("Nodo de detección de pista iniciado")
        self.error_publisher = self.create_publisher(Float32, "/pista/error_lateral", 10)
        # Sincronizador
        self.sync = message_filters.ApproximateTimeSynchronizer([self.color_sub, self.depth_sub], queue_size=10, slop=0.1)

        # Función que se ejecuta cuando llegan color y profundidad sincronizados
        self.sync.registerCallback(self.image_callback)

    def image_callback(self, color_msg, depth_msg):
        # Convertir mensajes ROS a imágenes OpenCV
        color_frame = imgmsg_to_cv2(color_msg)
        depth_frame = imgmsg_to_cv2_16uc1(depth_msg)
        # Aplicar mapa de color (colormap) a la imagen de profundidad para visualizarla mejor
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_frame, alpha=0.03), cv2.COLORMAP_JET)

        # Aquí usamos tu función actual
        resultado, detectado = detect_lines(color_frame, depth_frame)
        images = np.hstack((color_frame, depth_colormap))
        cv2.imshow('RealSense D435 - Color | Profundidad', images)

        cv2.imshow("Pista", detectado)
        if cv2.waitKey(1) & (0xFF == ord("q") or 0xFF == 27):  # 27 es la tecla ESC
            self.get_logger().info("Cerrando nodo de detección de pista")

# Replace br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
def imgmsg_to_cv2(msg: Image) -> np.ndarray:
    img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
    return img

def imgmsg_to_cv2_16uc1(msg: Image) -> np.ndarray:
    if msg.encoding != '16UC1':
        raise ValueError(f"Expected encoding '16UC1', got '{msg.encoding}'")
    
    # Handle endianness if the publisher is big-endian, otherwise standard little-endian uint16
    dtype = np.dtype(np.uint16).newbyteorder('>' if msg.is_bigendian else '<')
    
    # Use strides in case the publisher pads rows (msg.step > msg.width * 2)
    img = np.ndarray(
        shape=(msg.height, msg.width),
        dtype=dtype,
        buffer=msg.data,
        strides=(msg.step, 2)
    )
    return img.copy()  # .copy() ensures a contiguous, writable array for OpenCV 5

def constrain(val, min_val):
    if val < min_val: return min_val
    return val

# Función que transforma la perspectiva de la imagen para ver el suelo con proporciones más claras
def transformacion_perspectiva(frame):
    h, w = frame.shape[:2]

    # Puntos aproximados.
    pista = np.float32([[w * 0.25, h * 0.4], [w * 0.75, h * 0.4], [w * 1, h], [w * 0.0, h]])
    perspectiva = np.float32([[w * 0.20, 0], [w * 0.80, 0], [w * 0.80, h], [w * 0.20, h]])
    mapeo = cv2.getPerspectiveTransform(pista, perspectiva)

    frame_t = cv2.warpPerspective(frame, mapeo, (w, h))
    return frame_t, w, h

# Función para calcular la posición y ángulo relativo que se tiene con respecto a la pista
def calcular_pose_relativa(izq, der, ancho, alto, pendiente_referencia=0.0):
    # Signos de posicion: negativo = robot a la izquierda; positivo = robot a la derecha
    # La orientacion se expresa respecto de la calibracion inicial.
    y_ref = alto * 0.80
    x_robot = ancho / 2

    # Posicion de ambas lineas a la altura de referencia
    x_izq = izq["a"] * y_ref + izq["b"]
    x_der = der["a"] * y_ref + der["b"]

    if x_der <= x_izq:
        return None

    # Centro y ancho local de la pista
    x_centro = float((x_izq + x_der) / 2)
    ancho_pista = float(x_der - x_izq)

    # Posicion relativa del robot
    error_px = float(x_robot - x_centro)
    error_normalizado = float(error_px / (ancho_pista / 2))

    # Orientacion del centro de la pista
    pendiente_centro = float((izq["a"] + der["a"]) / 2)

    angulo_centro = np.arctan(pendiente_centro)
    angulo_referencia = np.arctan(pendiente_referencia)

    angulo_relativo = float(angulo_centro - angulo_referencia)

    # Diferencia entre las pendientes de los limites
    diferencia_pendientes = float(der["a"] - izq["a"])

    return {
        "error_lat_px": error_px,
        "error_lat": error_normalizado,
        "centro_pista_px": x_centro,
        "ancho_pista_px": ancho_pista,
        "orien_rad": angulo_relativo,
        "orien_grados": float(np.degrees(angulo_relativo)),
        "pend_centro": pendiente_centro,
        "dif_pends": diferencia_pendientes
    }

# Función que toma las líneas detectadas y define 1 línea detectada por sector
def agrupar_lineas(segmentos, alto):
    if segmentos is None:
        return []

    # Convertir todos los segmentos a arreglos NumPy
    s = np.zeros((len(segmentos), 4), dtype=np.float32)
    for i in range(len(segmentos)):
        s[i][0], s[i][1], s[i][2], s[i][3] = np.array(segmentos[i])
    x_1, y_1, x_2, y_2 = s.T
    d_y = y_2 - y_1
    d_y[d_y == 0] = 1e-6  # Evitar división por cero
    d_x = x_2 - x_1
    longitud = np.hypot(d_x, d_y)

    # # Filtrar segmentos horizontales y demasiado cortos
    # validos = (np.abs(d_y) >= 1) & (longitud >= MIN_LONGITUD)
    # x_1, y_1, d_y, d_x = x_1[validos], y_1[validos], d_y[validos], d_x[validos]

    if len(d_x) == 0:
        return []

    # Posicion y pendiente de cada segmento
    pendientes = d_x / d_y
    y_ref = alto * 0.80
    posiciones = x_1 + pendientes * (y_ref - y_1)

    # Histograma: cada bin representa 5 pixeles
    x_min = np.floor(posiciones.min() / p.ANCHO_GRUPO) * p.ANCHO_GRUPO
    indices = np.floor((posiciones - x_min) / p.ANCHO_GRUPO).astype(int)

    cantidad_bins = constrain(indices.max(), 0) + 1

    # Acumular cantidad, posiciones y pendientes
    conteo = np.bincount(indices, minlength=cantidad_bins)
    suma_x = np.bincount(indices, weights=posiciones, minlength=cantidad_bins)
    suma_m = np.bincount(indices, weights=pendientes, minlength=cantidad_bins)

    # Detectar zonas con suficientes segmentos
    activos = np.where(conteo >= p.MIN_SEGMENTOS_GRUPO)[0]

    lineas = []

    for i in activos:
        x_ref = float(suma_x[i] / conteo[i])
        m = float(suma_m[i] / conteo[i])

        a = float(m)
        b = float(x_ref - a * y_ref)

        lineas.append({"a": a, "b": b, "x_ref": x_ref, "pendiente": m, "cantidad_segmentos": int(conteo[i]), "ancho_grupo": float(p.ANCHO_GRUPO)})

    return sorted(lineas, key=lambda l: l["x_ref"])

# Función encargada de dibujar la línea
def dibujar_linea(frame, linea, color, etiqueta):
    h, w = frame.shape[:2]
    y_1, y_2 = int(h * 0.40), int(h * 0.98)
    x_1 = int(linea["a"] * y_1 + linea["b"])
    x_2 = int(linea["a"] * y_2 + linea["b"])

    # Evita dibujar coordenadas fuera de la frame.
    x_1 = int(np.clip(x_1, 0, w - 1))
    x_2 = int(np.clip(x_2, 0, w - 1))

    cv2.line(frame, (x_1, y_1), (x_2, y_2), color, 3)
    cv2.putText(frame, etiqueta, (x_2, max(20, y_2 - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

# Función encargada de detectar y procesar las líneas
def detect_lines(frame, depth_frame):
    vista, w, h = transformacion_perspectiva(frame)
    # Líneas blancas
    blanco_min = np.array([180, 180, 180])
    blanco_max = np.array([250, 250, 250])

    mascara = cv2.inRange(vista, blanco_min, blanco_max)

    # Suavizado
    mascara_s = cv2.GaussianBlur(mascara, (5, 5), 0)
    bordes = cv2.Canny(mascara_s, 50, 150)

    segmentos = cv2.HoughLinesP(bordes, 1, np.pi / 180, threshold=60, minLineLength=80, maxLineGap=40)

    lineas = agrupar_lineas(segmentos, h)
    for linea in lineas:
        distancia = obtener_profundidad_linea(linea, depth_frame, w, h)

        linea["distancia"] = distancia
    salida = vista.copy()
    centro_frame = w / 2

    cv2.line(salida, (int(centro_frame), 0), (int(centro_frame), h), (255, 0, 0), 1)

    # Dibuja todas las lineas representativas.
    for i, linea in enumerate(lineas):
        dibujar_linea(salida, linea, (255, 180, 0), f"L{i + 1} ({linea['cantidad_segmentos']})")

    izquierdas = [l for l in lineas if l["x_ref"] < centro_frame]
    derechas = [l for l in lineas if l["x_ref"] > centro_frame]

    opciones = [(izq, der) for izq in izquierdas for der in derechas if der["x_ref"] - izq["x_ref"] > w * 0.15]

    resultado = {"hay_pareja": False, "desviacion_px": None, "lineas_detectadas": len(lineas)}

    if opciones:
        # Primera aproximacion: pareja que encierra el centro de la frame con el menor ancho suficiente.
        izq, der = min(opciones, key=lambda p: p[1]["x_ref"] - p[0]["x_ref"])

        pose = calcular_pose_relativa(izq, der, w, h)

        if pose is not None:
            resultado.update(pose)

            texto = (f"Posicion: {pose['error_lat']:+.2f} | " f"Orientacion: {pose['orien_grados']:+.1f} deg")

            cv2.putText(salida, texto, (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

        centro_pista = (izq["x_ref"] + der["x_ref"]) / 2
        desviacion = centro_pista - centro_frame

        dibujar_linea(salida, izq, (0, 255, 0), "Izquierda")
        dibujar_linea(salida, der, (0, 255, 255), "Derecha")

        cv2.circle(salida, (int(np.clip(centro_pista, 0, w - 1)), int(h * 0.80)), 6, (0, 0, 255), -1)

        resultado.update({"hay_pareja": True, "desviacion_px": float(desviacion), "x_izquierda": izq["x_ref"], "x_derecha": der["x_ref"], "centro_pista": centro_pista})

        texto = f"Desviacion: {desviacion:.1f} px"
        cv2.putText(salida, texto, (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    else:
        cv2.putText(salida, "No se encontro una pareja valida", (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    resultado["lineas"] = lineas
    return resultado, salida

# Función que detecta si el robot está acercandose o cruzando una línea de pista y decide que hacer en dicho caso
def evaluar_recuperacion(lineas, frame_original):
    if not lineas:
        return {"modo": "sin_lineas", "accion": None}

    # Analizar las pendientes de las lineas detectadas
    pendientes = np.array([linea["a"] for linea in lineas])

    # Pendiente promedio de todas las lineas visibles
    pendiente_promedio = np.mean(pendientes)

    # La pendiente se expresa como dx/dy, los valores grandes corresponden a lineas mas horizontales.
    pendiente_horizontal = 2.0

    # CASO 1: aparece una linea casi horizontal
    if np.max(np.abs(pendientes)) > pendiente_horizontal:
        hay_pasto, proporcion_verde, _ = hay_verde(frame_original)

        if hay_pasto:
            accion = "derecha"
        else:
            accion = "izquierda"

        return {
            "modo": "linea_frontal",
            "accion": accion,
            "hay_verde": hay_pasto,
            "proporcion_verde": proporcion_verde,
            "pendiente_promedio": float(pendiente_promedio)
        }

    # CASO 2: deteccion normal
    return {
        "modo": "normal",
        "accion": None,
        "pendiente_promedio": float(pendiente_promedio)
    }

# Función auxiliar que, en caso de línea horizontal, define el lado al que ir según si hay pasto enfrente o no
def hay_verde(frame, proporcion_minima=0.08):
    # Retorna: hay_pasto: True o False; proporcion: porcentaje de pixeles verdes; mascara: mascara utilizada para depuracion
    h, w = frame.shape[:2]

    # Zona frontal de interes
    roi = frame[int(h * 0.35):int(h * 0.75), int(w * 0.20):int(w * 0.80)]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    # Rango inicial para verde
    verde_min = np.array([35, 40, 40])
    verde_max = np.array([90, 255, 255])

    mascara_verde = cv2.inRange(hsv, verde_min, verde_max)
    proporcion = np.count_nonzero(mascara_verde) / mascara_verde.size
    hay_pasto = proporcion >= proporcion_minima

    return hay_pasto, proporcion, mascara_verde

# Función encargada de ver la distancia de la línea con respecto a la cámara
def obtener_profundidad_linea(linea, depth_frame, ancho, alto):
    distancias = []

    # Solo se analiza la zona inferior de la imagen, que corresponde a la zona cercana al robot.
    y_inicio = int(alto * 0.70)
    y_final = int(alto * 0.93)

    for y in range(y_inicio, y_final, 5):
        x = int(linea["a"] * y + linea["b"])
        if 0 <= x < ancho:
            distancia = depth_frame.get_distance(x, y)

            # Ignorar valores inválidos o fuera de rango
            if 0.15 < distancia < 10.0:
                distancias.append(distancia)
    if not distancias:
        return None
    return float(np.median(distancias))

def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectionNode()
    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()