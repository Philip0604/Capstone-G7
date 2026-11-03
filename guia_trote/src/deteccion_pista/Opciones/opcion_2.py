import cv2
import numpy as np
import os


in_path = os.path.join("videos", "input")
out_path = os.path.join("videos", "output")
VIDEO_INPUT = os.path.join(in_path, "Pista_curva.mp4")
VIDEO_OUTPUT = os.path.join(out_path, "Pista_curva.mp4")


def detect_lane_lines(frame):

    # -----------------------------
    # 1. Convertir a HSV
    # -----------------------------
    lower_white = np.array([160, 160, 160])
    upper_white = np.array([250, 250, 250])
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # -----------------------------
    # 2. Máscara para líneas blancas
    # -----------------------------
    # lower_white = np.array([0, 0, 80])
    # upper_white = np.array([250, 20, 100])

    mask = cv2.inRange(
        frame,
        lower_white,
        upper_white
    )

    # -----------------------------
    # 3. Limpiar máscara
    # -----------------------------
    kernel = np.ones((5, 5), np.uint8)

    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_OPEN,
        kernel
    )

    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_CLOSE,
        kernel
    )

    # -----------------------------
    # 4. Canny
    # -----------------------------
    edges = cv2.Canny(
        mask,
        50,
        150
    )

    # -----------------------------
    # 5. Región de interés
    # -----------------------------
    height, width = frame.shape[:2]

    roi_mask = np.zeros_like(edges)

    polygon = np.array([
        [
            (0, height),
            (width, height),
            (int(width * 0.65), int(height * 0.45)),
            (int(width * 0.35), int(height * 0.45))
        ]
    ])

    cv2.fillPoly(
        roi_mask,
        polygon,
        255
    )

    edges_roi = cv2.bitwise_and(
        edges,
        roi_mask
    )

    # -----------------------------
    # 6. Hough
    # -----------------------------
    lines = cv2.HoughLinesP(
        edges_roi,
        rho=1,
        theta=np.pi / 180,
        threshold=50,
        minLineLength=40,
        maxLineGap=30
    )

    output = frame.copy()

    if lines is not None:

        for line in lines:

            x1, y1, x2, y2 = line[0]

            cv2.line(
                output,
                (x1, y1),
                (x2, y2),
                (0, 0, 255),
                3
            )

    return output, mask, edges_roi


# =====================================
# VIDEO
# =====================================

cap = cv2.VideoCapture(VIDEO_INPUT)

if not cap.isOpened():
    raise RuntimeError(
        f"No se pudo abrir {VIDEO_INPUT}"
    )


width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = cap.get(cv2.CAP_PROP_FPS)

fourcc = cv2.VideoWriter_fourcc(*"mp4v")

out = cv2.VideoWriter(
    VIDEO_OUTPUT,
    fourcc,
    fps,
    (width, height)
)


while True:

    ret, frame = cap.read()

    if not ret:
        break

    result, mask, edges = detect_lane_lines(frame)

    out.write(result)

    cv2.imshow(
        "Deteccion de pistas",
        result
    )

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break


cap.release()
out.release()
cv2.destroyAllWindows()