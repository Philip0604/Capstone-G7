import cv2
from pathlib import Path


VIDEO_DIR = Path("videos/input")
OUTPUT_DIR = Path("datasets/det_pista/images")


VIDEO_NAMES = [
    "Pista_curva.mp4",
    "Pista_recta.mp4",
    "Pista_persona_pasto.mp4",
    "Pista_persona.mp4",
    "Pista_primera_linea.mp4",
    "Pista_primera.mp4",
    "Pista_sentido_contrario.mp4",
    "Pista_zigzag_recta.mp4",
    "Pista_zigzag.mp4"
]

# Guardar 1 frame cada N frames
FRAME_INTERVAL = 15


OUTPUT_DIR.mkdir(
    parents=True,
    exist_ok=True
)


for video_name in VIDEO_NAMES:

    video_path = VIDEO_DIR / video_name

    cap = cv2.VideoCapture(str(video_path))

    if not cap.isOpened():
        print(f"No se pudo abrir: {video_path}")
        continue

    frame_number = 0
    saved_frames = 0

    video_id = video_path.stem

    while True:

        ret, frame = cap.read()

        if not ret:
            break

        if frame_number % FRAME_INTERVAL == 0:

            filename = (
                f"{video_id}_{saved_frames:05d}.jpg"
            )

            output_path = OUTPUT_DIR / filename

            cv2.imwrite(
                str(output_path),
                frame
            )

            saved_frames += 1

        frame_number += 1

    cap.release()

    print(
        f"{video_name}: "
        f"{saved_frames} frames guardados"
    )