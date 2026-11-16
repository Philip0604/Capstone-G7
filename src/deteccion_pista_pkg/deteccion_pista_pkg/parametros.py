"""Parametros de la deteccion de pista (ajustar en terreno)."""

# --- Resolucion de procesamiento -------------------------------------------
# La imagen se reduce a este ancho antes de procesar (320 es ~4x mas barato que 640).
ANCHO_PROCESO = 320

# --- Region de interes y vista cenital --------------------------------------
# Fraccion de la altura desde donde empieza el suelo que se analiza.
Y_SUPERIOR = 0.40
# Trapecio (fracciones del ancho) que se lleva a un rectangulo en vista cenital.
TRAPECIO_SUP_IZQ = 0.25
TRAPECIO_SUP_DER = 0.75
TRAPECIO_INF_IZQ = 0.00
TRAPECIO_INF_DER = 1.00
# Altura (fraccion) donde se mide la posicion de las lineas para el error lateral.
Y_REFERENCIA = 0.90

# --- Mascara de lineas blancas ----------------------------------------------
TAM_FONDO = 31          # tamano (px, impar) de la ventana para estimar el brillo del asfalto
CONTRASTE_MIN = 18      # cuanto mas clara que su entorno debe ser la linea
BRILLO_MIN = 120        # brillo (V de HSV) minimo absoluto
SATURACION_MAX = 70     # las lineas blancas son poco saturadas (descarta pasto seco)

# --- Busqueda de lineas -------------------------------------------------------
HOUGH_UMBRAL = 15            # votos minimos de Hough
HOUGH_LARGO_MIN = 0.10       # largo minimo de segmento (fraccion del alto)
HOUGH_SALTO_MAX = 0.12       # hueco maximo que se "salta" dentro de una linea (fraccion del alto)
ANGULO_MAX_LINEA = 55.0      # grados respecto de la vertical en vista cenital
TOL_UNION = 0.05             # fraccion del ancho para unir segmentos de la misma linea
TOL_UNION_GRADOS = 12.0
SOPORTE_MIN = 0.30           # suma de largos de segmentos (fraccion del alto)
LARGO_MIN_LINEA = 0.25       # extension vertical minima de una linea (fraccion del alto)
MAX_LINEAS = 6

# --- Seleccion del par --------------------------------------------------------
ANCHO_PISTA_MIN = 0.15        # fraccion del ancho (vista cenital)
ANCHO_PISTA_MAX = 0.95
MAX_DIF_ANGULO_PAR = 35.0     # grados: las dos lineas deben ser casi paralelas en vista cenital

# --- Normalizacion de errores -------------------------------------------------
# El Arduino recorta los errores a [-1, 1]; el error angular se divide por este angulo.
ANGULO_MAX_GRADOS = 30.0
