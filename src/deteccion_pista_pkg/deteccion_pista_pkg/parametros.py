import os


VIDEO_ENTRADA = os.path.join("videos", "input", "Pista_zigzag.mp4")
VIDEO_SALIDA = os.path.join("videos", "output", "Pista_zigzag_perspectiva.mp4")

UMBRAL_BLANCO = 180
TOLERANCIA_GRUPOS = 35
MIN_LONGITUD = 35

ANCHO_GRUPO = 15  # Pixeles
MIN_SEGMENTOS_GRUPO = 5