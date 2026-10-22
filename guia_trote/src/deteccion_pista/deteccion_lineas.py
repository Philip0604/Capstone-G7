import cv2
import numpy as np
import os

in_path = os.path.join("videos", "input")
out_path = os.path.join("videos", "output")
VIDEO_INPUT = os.path.join(in_path, "Pista_zigzag.mp4")
VIDEO_OUTPUT = os.path.join(out_path, "Pista_zigzag_perspectiva.mp4")

UMBRAL_BLANCO = 180
TOLERANCIA_GRUPOS = 35
MIN_LONGITUD = 35

def transformacion_perspectiva(frame):
    h, w = frame.shape[:2]

    # Puntos aproximados.
    pista = np.float32([[w * 0.25, h * 0.4], [w * 0.75, h * 0.4], [w * 1, h], [w * 0.0, h]])
    perspectiva = np.float32([[w * 0.20, 0], [w * 0.80, 0], [w * 0.80, h], [w * 0.20, h]])
    mapeo = cv2.getPerspectiveTransform(pista, perspectiva)

    frame_t = cv2.warpPerspective(frame, mapeo, (w, h))
    return frame_t


def detect_lines(frame):
    # Líneas blancas
    blanco_min = np.array([180, 180, 180])
    blanco_max = np.array([250, 250, 250])

    mask = cv2.inRange(frame, blanco_min, blanco_max)

    # Suavizado
    mask_s = cv2.GaussianBlur(mask, (5, 5), 0)
    edges = cv2.Canny(mask_s, 50, 150)

    lineas = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=60, minLineLength=80, maxLineGap=40)

    resultado = frame.copy()

    if lineas is not None:
        for linea in lineas:
            x1, y1, x2, y2 = linea[0]

            # Pendiente
            if x2 != x1:
                pendiente = ((y2 - y1) / (x2 - x1))

                # Filtrar líneas aproximadamente verticales/horizontales según el caso
                if abs(pendiente) > 0.7:
                    cv2.line(resultado, (x1, y1), (x2, y2), (0, 255, 0), 3)
    return resultado

def agrupar_lineas(segmentos, alto):
    if segmentos is None:
        return []

    y_ref = alto * 0.80
    candidatos = []

    for s in segmentos[:, 0]:
        x1, y1, x2, y2 = map(float, s)

        if abs(y2 - y1) < 1:
            continue
        if np.hypot(x2 - x1, y2 - y1) < MIN_LONGITUD:
            continue

        pendiente = (x2 - x1) / (y2 - y1)
        x_ref = x1 + pendiente * (y_ref - y1)

        candidatos.append((x_ref, [x1, y1, x2, y2]))

    candidatos.sort(key=lambda item: item[0])
    grupos = []

    for x_ref, extremos in candidatos:
        if (not grupos or abs(x_ref - np.mean([p[0] for p in grupos[-1]])) > TOLERANCIA_GRUPOS):
            grupos.append([])

        grupos[-1].append((x_ref, extremos))

    lineas = []
    for grupo in grupos:
        puntos = []
        for _, (x1, y1, x2, y2) in grupo:
            puntos.extend([[x1, y1], [x2, y2]])

        puntos = np.array(puntos, dtype=np.float32)
        vx, vy, x0, y0 = cv2.fitLine(puntos, cv2.DIST_L2, 0, 0.01, 0.01).flatten()

        if abs(vy) < 1e-5:
            continue

        a = vx / vy
        b = x0 - a * y0

        lineas.append({"a": float(a), "b": float(b), "x_ref": float(a * y_ref + b), "cantidad_segmentos": len(grupo)})
    return sorted(lineas, key=lambda l: l["x_ref"])


def dibujar_linea(imagen, linea, color, etiqueta):
    h, w = imagen.shape[:2]
    y1, y2 = int(h * 0.40), int(h * 0.98)
    x1 = int(linea["a"] * y1 + linea["b"])
    x2 = int(linea["a"] * y2 + linea["b"])

    # Evita dibujar coordenadas fuera de la imagen.
    x1 = int(np.clip(x1, 0, w - 1))
    x2 = int(np.clip(x2, 0, w - 1))

    cv2.line(imagen, (x1, y1), (x2, y2), color, 3)
    cv2.putText(imagen, etiqueta, (x2, max(20, y2 - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)


def procesar_frame(imagen):
    h, w = imagen.shape[:2]
    vista, _ = transformacion_perspectiva(imagen)

    gris = cv2.cvtColor(vista, cv2.COLOR_BGR2GRAY)
    mascara = cv2.inRange(gris, UMBRAL_BLANCO, 255)

    mascara = cv2.morphologyEx(mascara, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

    bordes = cv2.Canny(mascara, 50, 150)

    segmentos = cv2.HoughLinesP(bordes, 1, np.pi / 180, threshold=35, minLineLength=MIN_LONGITUD, maxLineGap=35)

    lineas = agrupar_lineas(segmentos, h)
    salida = vista.copy()
    centro_imagen = w / 2

    cv2.line(salida, (int(centro_imagen), 0), (int(centro_imagen), h), (255, 0, 0), 1)

    # Dibuja todas las lineas representativas.
    for i, linea in enumerate(lineas):
        dibujar_linea(salida, linea, (255, 180, 0), f"L{i + 1} ({linea['cantidad_segmentos']})")

    izquierdas = [l for l in lineas if l["x_ref"] < centro_imagen]
    derechas = [l for l in lineas if l["x_ref"] > centro_imagen]

    opciones = [(izq, der) for izq in izquierdas for der in derechas if der["x_ref"] - izq["x_ref"] > w * 0.15]

    resultado = {"hay_pareja": False, "desviacion_px": None, "lineas_detectadas": len(lineas)}

    if opciones:
        # Primera aproximacion: pareja que encierra el centro
        # de la imagen con el menor ancho suficiente.
        izq, der = min(opciones, key=lambda p: p[1]["x_ref"] - p[0]["x_ref"])

        centro_pista = (izq["x_ref"] + der["x_ref"]) / 2
        desviacion = centro_pista - centro_imagen

        dibujar_linea(salida, izq, (0, 255, 0), "Izquierda")
        dibujar_linea(salida, der, (0, 255, 255), "Derecha")

        cv2.circle(salida, (int(np.clip(centro_pista, 0, w - 1)), int(h * 0.80)), 6, (0, 0, 255), -1)

        resultado.update({"hay_pareja": True, "desviacion_px": float(desviacion), "x_izquierda": izq["x_ref"], "x_derecha": der["x_ref"], "centro_pista": centro_pista})

        texto = f"Desviacion: {desviacion:.1f} px"
        cv2.putText(salida, texto, (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    else:
        cv2.putText(salida, "No se encontro una pareja valida", (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    resultado["lineas"] = lineas
    return resultado, salida, mascara, bordes


def main():
    cap = cv2.VideoCapture(VIDEO_ENTRADA)

    if not cap.isOpened():
        raise RuntimeError(f"No se pudo abrir: {VIDEO_ENTRADA}")

    fps = cap.get(cv2.CAP_PROP_FPS)
    if not np.isfinite(fps) or fps <= 0:
        fps = 30.0

    ancho = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    alto = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    salida_video = cv2.VideoWriter(
        VIDEO_SALIDA,
        cv2.VideoWriter_fourcc(*"mp4v"),
        fps, (ancho, alto)
    )

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            resultado, vista, mascara, bordes = procesar_frame(frame)
            salida_video.write(vista)

            print(f"Lineas: {resultado['lineas_detectadas']} | " f"Pareja: {resultado['hay_pareja']} | " f"Desviacion: {resultado['desviacion_px']}")

            cv2.imshow("Vista perspectiva", vista)
            cv2.imshow("Mascara blanca", mascara)
            cv2.imshow("Bordes Canny", bordes)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    finally:
        cap.release()
        salida_video.release()
        cv2.destroyAllWindows()

    print(f"Video guardado en: {VIDEO_SALIDA}")


if __name__ == "__main__":
    main()


cap = cv2.VideoCapture(VIDEO_INPUT)

if not cap.isOpened():
    raise RuntimeError("No se pudo abrir el video")


width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = cap.get(cv2.CAP_PROP_FPS)

fourcc = cv2.VideoWriter_fourcc(*"mp4v")

out = cv2.VideoWriter(VIDEO_OUTPUT, fourcc, fps, (width, height))


while True:
    ret, frame = cap.read()

    if not ret:
        break

    # Bird's eye view
    warped = perspective_transform(frame)

    # Detectar líneas
    detected = detect_lines(warped)

    out.write(detected)

    cv2.imshow("Pista", detected)

    cv2.imshow("Normal", frame)
    cv2.imshow("Bird Eye", warped)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break


cap.release()
out.release()

cv2.destroyAllWindows()


