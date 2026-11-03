import cv2
import numpy as np
from pathlib import Path

# CONFIGURACIÓN
DATASET_DIR = Path("datasets/det_pista")

IMAGE_DIR = DATASET_DIR / "images"
LABEL_DIR = DATASET_DIR / "labels"

CLASS_ID = 0
CLASS_NAME = "linea_pista"

WINDOW_NAME = "Anotador - YOLO Segmentation"



# VARIABLES DE ANOTACIÓN

# Polígonos que ya fueron terminados
polygons = []

# Puntos del polígono que estamos dibujando actualmente, para detectar múltiples polígonos en una imagen si es necesario
current_polygon = []

# Indica si el polígono actual está cerrado
polygon_closed = False

# CALLBACK DEL MOUSE
def mouse_callback(event, x, y, flags, param):
    global current_polygon
    global polygon_closed

    # CLICK IZQUIERDO
    # Agregar punto al polígono actual
    if event == cv2.EVENT_LBUTTONDOWN:
        if not polygon_closed:
            current_polygon.append((x, y))
            print(f"Punto agregado: ({x}, {y})")

    # CLICK DERECHO
    # Cerrar polígono actual
    elif event == cv2.EVENT_RBUTTONDOWN:
        if len(current_polygon) >= 3:
            polygon_closed = True
            print(f"Polígono cerrado ", f"({len(current_polygon)} puntos)")
        else:
            print("Se necesitan al menos 3 puntos ""para cerrar un polígono."
            )

# INICIAR NUEVO POLÍGONO
def start_new_polygon():
    global current_polygon
    global polygon_closed
    # Si existe un polígono cerrado,
    # lo agregamos a la lista de polígonos.
    if polygon_closed and len(current_polygon) >= 3:
        polygons.append(current_polygon.copy())
        print(f"Polígono guardado en memoria. "f"Total: {len(polygons)}")
    # Comenzamos un nuevo polígono
    current_polygon = []
    polygon_closed = False

# REINICIAR POLÍGONO ACTUAL
def reset_current_polygon():
    global current_polygon
    global polygon_closed
    current_polygon = []
    polygon_closed = False
    print("Polígono actual eliminado.")

# ELIMINAR TODA LA ANOTACIÓN
def reset_all_polygons():
    global polygons
    global current_polygon
    global polygon_closed
    polygons = []
    current_polygon = []
    polygon_closed = False
    print("Todas las anotaciones fueron eliminadas.")

# NORMALIZAR PUNTOS
def normalize_polygon(polygon,width,height):
    normalized = []
    for x, y in polygon:
        x_norm = x / width
        y_norm = y / height
        normalized.append(x_norm)
        normalized.append(y_norm)
    return normalized

# GUARDAR ANOTACIONES
def save_annotation(image_path,width,height):
    # Si hay un polígono cerrado que todavía está en
    # current_polygon, lo agregamos.
    if polygon_closed and len(current_polygon) >= 3:
        if (len(polygons) == 0 or polygons[-1] != current_polygon):
            polygons.append(current_polygon.copy())
    
    # Verificar que existan polígonos
    if len(polygons) == 0:
        print("No hay polígonos para guardar.")
        return False

    # Archivo .txt
    label_path = (LABEL_DIR /f"{image_path.stem}.txt")
    with open(label_path, "w") as file:
        # Cada polígono corresponde a UNA línea
        # dentro del archivo YOLO.
        for polygon in polygons:
            normalized = normalize_polygon(polygon,width,height)
            line = [str(CLASS_ID)]
            for value in normalized:
                line.append(f"{value:.6f}")
            file.write(" ".join(line) + "\n")
    print(f"Anotación guardada: {label_path}")
    print(f"Polígonos guardados: {len(polygons)}")
    return True

# DIBUJAR ANOTACIONES
def draw_annotation(frame):
    output = frame.copy()

    # DIBUJAR POLÍGONOS TERMINADOS
    for polygon_index, polygon in enumerate(polygons):
        # Puntos
        for point_index, (x, y) in enumerate(polygon):
            cv2.circle(
                output,
                (x, y),
                5,
                (0, 0, 255),
                -1
            )
            cv2.putText(
                output,
                str(point_index + 1),
                (x + 8, y - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 0, 255),
                1
            )

        # Líneas
        for i in range(len(polygon) - 1):
            cv2.line(
                output,
                polygon[i],
                polygon[i + 1],
                (0, 255, 0),
                2
            )

        # Cerrar polígono
        if len(polygon) >= 3:
            cv2.line(
                output,
                polygon[-1],
                polygon[0],
                (0, 255, 0),
                2
            )
        
        # Relleno semitransparente
        overlay = output.copy()
        polygon_array = np.array(
            polygon,
            dtype=np.int32
        )
        cv2.fillPoly(
            overlay,
            [polygon_array],
            (0, 255, 0)
        )
        output = cv2.addWeighted(
            overlay,
            0.20,
            output,
            0.80,
            0
        )
    
    # DIBUJAR POLÍGONO ACTUAL
    if len(current_polygon) > 0:    
        # Puntos
        for point_index, (x, y) in enumerate(
            current_polygon
        ):
            cv2.circle(
                output,
                (x, y),
                5,
                (255, 0, 0),
                -1
            )
            cv2.putText(
                output,
                str(point_index + 1),
                (x + 8, y - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 0, 0),
                1
            )
        
        # Líneas entre puntos
        for i in range(
            len(current_polygon) - 1
        ):
            cv2.line(
                output,
                current_polygon[i],
                current_polygon[i + 1],
                (255, 0, 0),
                2
            )
        
        # Si está cerrado, dibujar línea final
        if polygon_closed and len(current_polygon) >= 3:
            cv2.line(
                output,
                current_polygon[-1],
                current_polygon[0],
                (255, 0, 0),
                2
            )
    return output


# MOSTRAR AYUDA
def show_help(frame):
    instructions = [
        "Click izquierdo: agregar punto",
        "Click derecho: cerrar poligono",
        "ENTER: nuevo poligono",
        "R: reiniciar poligono actual",
        "A: borrar todos",
        "S: guardar",
        "N: siguiente",
        "P: anterior",
        "Q / ESC: salir"
    ]
    y = 25
    for text in instructions:
        cv2.putText(
            frame,
            text,
            (10, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            1,
            cv2.LINE_AA
        )
        y += 23

# PREPARAR DIRECTORIOS
IMAGE_DIR.mkdir(
    parents=True,
    exist_ok=True
)
LABEL_DIR.mkdir(
    parents=True,
    exist_ok=True
)
# BUSCAR IMÁGENES
image_extensions = [
    "*.jpg",
    "*.jpeg",
    "*.png"
]
image_paths = []
for extension in image_extensions:
    image_paths.extend(
        IMAGE_DIR.glob(extension)
    )
image_paths = sorted(image_paths)
if not image_paths:
    raise RuntimeError(
        f"No se encontraron imágenes en "
        f"{IMAGE_DIR}"
    )
print(
    f"Imágenes encontradas: "
    f"{len(image_paths)}"
)

# CREAR VENTANA
cv2.namedWindow(
    WINDOW_NAME,
    cv2.WINDOW_NORMAL
)
cv2.setMouseCallback(
    WINDOW_NAME,
    mouse_callback
)

# LOOP PRINCIPAL
index = 0
while 0 <= index < len(image_paths):
    if index < 40: continue
    image_path = image_paths[index]
    frame = cv2.imread(
        str(image_path)
    )
    if frame is None:
        print(
            f"No se pudo leer: "
            f"{image_path}"
        )
        index += 1
        continue
    height, width = frame.shape[:2]

    # Reiniciar anotación para este frame
    polygons = []
    current_polygon = []
    polygon_closed = False

    print()
    print("=" * 60)
    print(
        f"Imagen {index + 1}/"
        f"{len(image_paths)}"
    )
    print(
        f"Archivo: {image_path.name}"
    )
    print(
        f"Resolución: {width} x {height}"
    )
    print("=" * 60)

    while True:
        display = draw_annotation(
            frame.copy()
        )
        show_help(display)
        status = (
            f"Frame: {index + 1}/"
            f"{len(image_paths)}"
            f" | Poligonos: {len(polygons)}"
            f" | Puntos actuales: "
            f"{len(current_polygon)}"
        )
        cv2.putText(
            display,
            status,
            (10, height - 15),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
            cv2.LINE_AA
        )
        cv2.imshow(
            WINDOW_NAME,
            display
        )
        key = cv2.waitKey(20) & 0xFF
        
        # SALIR
        
        if key == ord("q") or key == 27:
            cv2.destroyAllWindows()
            raise SystemExit
        
        # REINICIAR POLÍGONO ACTUAL
        
        elif key == ord("r"):
            reset_current_polygon()
        
        # BORRAR TODO
        
        elif key == ord("a"):
            reset_all_polygons()
        
        # NUEVO POLÍGONO
        
        elif key == 13:
            if polygon_closed:
                start_new_polygon()
            else:
                print(
                    "Primero debes cerrar "
                    "el polígono actual."
                )
        
        # GUARDAR
        
        elif key == ord("s"):
            save_annotation(
                image_path,
                width,
                height
            )
        
        # SIGUIENTE
        
        elif key == ord("n"):
            saved = save_annotation(
                image_path,
                width,
                height
            )
            if saved:
                index += 1
                break
        # ANTERIOR
        elif key == ord("p"):
            index = max(
                0,
                index - 1
            )
            break

cv2.destroyAllWindows()