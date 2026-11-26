# Algoritmo de deteccion de pista (sin dependencias de ROS, se puede probar con videos).

# Flujo por frame (pensado para Raspberry Pi 4):
#   1. Se reduce la imagen a ANCHO_PROCESO pixeles de ancho (por defecto 320).
#   2. Mascara de "blanco": pixeles claros, poco saturados y mas claros que el
#      asfalto que los rodea (contraste local). Asi funciona con lineas gastadas
#      y con distinta iluminacion, sin un umbral fijo de BGR.
#   3. Se pasa la mascara (1 canal, barato) a vista cenital.
#      (se borran los pixeles rodeados de pasto: el pasto con sol parece "blanco").
#   4. Se buscan segmentos con Hough, se agrupan en lineas x = a*y + b y se
#      VALIDA cada linea: debe tener pixeles en casi todo su tramo y muchos mas
#      que el piso de al lado. Asi se descartan las "lineas" armadas con ruido.
#   5. Se elige, entre las lineas validas, el par izquierda/derecha que encierra
#      el centro de la imagen. Si no hay dos lineas validas NO hay par (no se
#      completa el par con una linea dudosa).
#   6. Las dos rectas se devuelven a la imagen original y se calculan:
#        - error lateral: 0 cuando el robot esta al centro entre las 2 lineas.
#        - error angular: 0 cuando ambas lineas tienen la misma pendiente,
#          una "/" y la otra "\\" (simetricas).

# Convencion de signos (coincide con codigo_motores.ino, donde
# correccion > 0 => rueda derecha mas rapida => el robot gira a la izquierda):
#   - error_lateral  > 0: el robot esta a la DERECHA del centro de la pista.
#   - error_angular  > 0: el robot apunta hacia la DERECHA respecto de la pista.
#   En ambos casos el robot debe girar a la izquierda.

import cv2
import numpy as np

from deteccion_pista_pkg import parametros as p


class DetectorPista:
    def __init__(self, ancho_entrada, alto_entrada):
        self.ancho_in = ancho_entrada
        self.alto_in = alto_entrada

        # Tamano de procesamiento (se conserva la proporcion de la camara)
        self.escala = p.ANCHO_PROCESO / float(ancho_entrada)
        self.w = int(p.ANCHO_PROCESO)
        self.h = int(round(alto_entrada * self.escala))

        # Region de interes: solo desde Y_SUPERIOR hacia abajo (suelo)
        self.y0 = int(self.h * p.Y_SUPERIOR)

        # Homografia (se calcula UNA vez, no en cada frame)
        w, h = self.w, self.h
        pista = np.float32([
            [w * p.TRAPECIO_SUP_IZQ, h * p.Y_SUPERIOR],
            [w * p.TRAPECIO_SUP_DER, h * p.Y_SUPERIOR],
            [w * p.TRAPECIO_INF_DER, h],
            [w * p.TRAPECIO_INF_IZQ, h],
        ])
        cenital = np.float32([[w * 0.20, 0], [w * 0.80, 0], [w * 0.80, h], [w * 0.20, h]])
        self.M = cv2.getPerspectiveTransform(pista, cenital)
        self.M_inv = cv2.getPerspectiveTransform(cenital, pista)
        # Zona de la vista cenital que la camara realmente ve (fuera de ella no hay datos:
        # no cuenta como "falta pintura" al validar una linea).
        roi = np.zeros((h, w), np.uint8)
        roi[int(h * p.Y_SUPERIOR):, :] = 255
        self.visible = cv2.warpPerspective(roi, self.M, (w, h), flags=cv2.INTER_NEAREST) > 0

        # Kernels precalculados
        self.k_fondo = (p.TAM_FONDO, p.TAM_FONDO)
        self.k_open = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        self.k_veg = (p.VEG_VENTANA, p.VEG_VENTANA)

        # Memoria del ultimo par (ayuda a que no "salte" entre lineas)
        self.x_izq_prev = None
        self.x_der_prev = None
        self.descartadas = 0

    # 1-2. Mascara de lineas blancas
    def mascara_blanco(self, pequena):
        roi = pequena[self.y0:, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        s = hsv[:, :, 1]
        v = hsv[:, :, 2]

        # Contraste local: brillo del pixel - brillo promedio de su entorno
        fondo = cv2.blur(v, self.k_fondo)
        contraste = cv2.subtract(v, fondo)

        m = (contraste > p.CONTRASTE_MIN) & (v > p.BRILLO_MIN) & (s < p.SATURACION_MAX)
        m &= ~self.mascara_vegetacion(hsv)
        m = m.astype(np.uint8) * 255
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, self.k_open)

        completa = np.zeros((self.h, self.w), np.uint8)
        completa[self.y0:, :] = m
        return completa

    def mascara_vegetacion(self, hsv):
        """True donde el entorno del pixel es mayormente pasto (verde o seco)."""
        h, s, v = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]
        verde = (h >= p.VEG_TONO_VERDE[0]) & (h <= p.VEG_TONO_VERDE[1]) & (s >= p.VEG_SAT_VERDE)
        seco = (h >= p.VEG_TONO_SECO[0]) & (h < p.VEG_TONO_SECO[1]) & (s >= p.VEG_SAT_SECO)
        veg = ((verde | seco) & (v >= 30)).astype(np.float32)
        return cv2.blur(veg, self.k_veg) > p.VEG_FRACCION_MAX

    # 4b. Validacion de una linea
    def calidad_linea(self, integral, linea):
        """
        (relleno, contraste) de la recta en la mascara cenital.
        integral: suma acumulada por filas de la mascara (para contar rapido).
        """
        h, w = self.h, self.w
        ys = np.arange(h)
        xs = linea["a"] * ys + linea["b"]

        def filas_con_pixeles(centro, radio):
            """(hay pixeles en la franja, la franja esta en la zona visible) por fila."""
            x0 = np.clip(np.round(centro - radio).astype(int), 0, w)
            x1 = np.clip(np.round(centro + radio).astype(int) + 1, 0, w)
            xc = np.round(centro).astype(int)
            ve = (xc >= 0) & (xc < w)
            ve[ve] = self.visible[ys[ve], xc[ve]]
            return (integral[ys, x1] - integral[ys, x0]) > 0, ve

        en_linea, ve = filas_con_pixeles(xs, w * p.BANDA_LINEA)
        filas = np.nonzero(en_linea & ve)[0]
        if len(filas) < 2:
            return 0.0, 0.0
        tramo = np.zeros(h, bool)
        tramo[filas[0]:filas[-1] + 1] = True
        relleno = float(en_linea[tramo & ve].mean())
        sep, ancho = w * p.SEPARACION_FONDO, w * p.ANCHO_FONDO
        # Lineas gruesas (o ensanchadas por la vista cenital): las franjas de fondo
        # se ponen fuera de la linea. Grosor = pixeles por fila en +-2*sep alrededor.
        x0 = np.clip(np.round(xs - 2 * sep).astype(int), 0, w)
        x1 = np.clip(np.round(xs + 2 * sep).astype(int) + 1, 0, w)
        filas_linea = tramo & ve & en_linea
        grosor = float(np.median((integral[ys, x1] - integral[ys, x0])[filas_linea]))
        sep = min(max(sep, 0.5 * grosor + ancho + 2), 2 * sep)
        fondo = 0.0
        for lado in (-1, 1):
            hay, ve_f = filas_con_pixeles(xs + lado * sep, ancho)
            if np.any(tramo & ve_f):
                fondo = max(fondo, float(hay[tramo & ve_f].mean()))
        return relleno, relleno / max(fondo, 0.05)

    # 4. Busqueda de lineas (Hough + agrupacion + validacion)
    def buscar_lineas(self, cenital):
        """
        Hough probabilistico sobre la mascara en vista cenital (sin Canny: la
        mascara ya es delgada) y agrupacion de segmentos colineales en lineas.
        maxLineGap permite "saltar" los tramos gastados de la pintura.
        """
        h, w = cenital.shape
        y_ref = h * p.Y_REFERENCIA
        segs = cv2.HoughLinesP(cenital, 1, np.pi / 180, threshold=p.HOUGH_UMBRAL,
                               minLineLength=int(h * p.HOUGH_LARGO_MIN),
                               maxLineGap=int(h * p.HOUGH_SALTO_MAX))
        if segs is None:
            return []

        s = segs.reshape(-1, 4).astype(np.float32)
        x1, y1, x2, y2 = s.T
        dy = y2 - y1
        dy[np.abs(dy) < 1e-3] = 1e-3
        a = (x2 - x1) / dy                          # x = a*y + b
        ang = np.degrees(np.arctan(a))              # 0 = vertical en vista cenital
        largo = np.hypot(x2 - x1, y2 - y1)
        x_ref = x1 + a * (y_ref - y1)

        validos = np.abs(ang) <= p.ANGULO_MAX_LINEA
        if not np.any(validos):
            return []
        orden = np.argsort(-largo[validos])
        idx_validos = np.nonzero(validos)[0][orden]

        # Agrupacion: cada segmento se une al grupo con posicion y angulo parecidos
        grupos = []
        tol_x = w * p.TOL_UNION
        for i in idx_validos:
            for g in grupos:
                if abs(x_ref[i] - g["x_ref"]) < tol_x and abs(ang[i] - g["ang"]) < p.TOL_UNION_GRADOS:
                    g["idx"].append(i)
                    break
            else:
                grupos.append({"x_ref": x_ref[i], "ang": ang[i], "idx": [i]})

        # suma acumulada por filas: cuenta pixeles en un tramo de fila en O(1)
        integral = np.zeros((h, w + 1), np.int32)
        np.cumsum(cenital > 0, axis=1, dtype=np.int32, out=integral[:, 1:])

        lineas = []
        self.descartadas = 0
        for g in grupos:
            idx = np.array(g["idx"])
            ys_g = np.concatenate([y1[idx], y2[idx]])
            xs_g = np.concatenate([x1[idx], x2[idx]])
            pesos = np.concatenate([largo[idx], largo[idx]])
            soporte = float(largo[idx].sum())
            cobertura = float(ys_g.max() - ys_g.min())
            if soporte < h * p.SOPORTE_MIN or cobertura < h * p.LARGO_MIN_LINEA:
                continue
            if cobertura > 1.0:
                aa, bb = np.polyfit(ys_g, xs_g, 1, w=pesos)
            else:
                aa, bb = float(a[idx[0]]), float(x1[idx[0]] - a[idx[0]] * y1[idx[0]])
            linea = {"a": float(aa), "b": float(bb), "x_ref": float(aa * y_ref + bb),
                     "soporte": soporte, "segmentos": int(len(idx))}
            linea["relleno"], linea["contraste"] = self.calidad_linea(integral, linea)
            solida = linea["relleno"] >= p.RELLENO_SOLIDO
            contrastada = linea["relleno"] >= p.RELLENO_MIN and linea["contraste"] >= p.CONTRASTE_MIN_LINEA
            if not (solida or contrastada):
                self.descartadas += 1   # "linea" armada con ruido
                continue
            lineas.append(linea)

        lineas.sort(key=lambda l: -l["soporte"])
        lineas = lineas[:p.MAX_LINEAS]
        return sorted(lineas, key=lambda l: l["x_ref"])

    # 5. Eleccion del par de lineas
    def elegir_par(self, lineas):
        w = self.w
        centro = w / 2.0
        izqs = [l for l in lineas if l["x_ref"] < centro]
        ders = [l for l in lineas if l["x_ref"] >= centro]
        opciones = []
        for i in izqs:
            for d in ders:
                ancho = d["x_ref"] - i["x_ref"]
                if not (w * p.ANCHO_PISTA_MIN <= ancho <= w * p.ANCHO_PISTA_MAX):
                    continue
                # En vista cenital las lineas de un carril son casi paralelas
                if abs(np.arctan(d["a"]) - np.arctan(i["a"])) > np.radians(p.MAX_DIF_ANGULO_PAR):
                    continue
                costo = ancho
                # Preferir el par cercano al del frame anterior
                if self.x_izq_prev is not None:
                    costo += 2.0 * (abs(i["x_ref"] - self.x_izq_prev) +
                                    abs(d["x_ref"] - self.x_der_prev))
                opciones.append((costo, i, d))
        if not opciones:
            return None
        _, izq, der = min(opciones, key=lambda o: o[0])
        return izq, der

    # 6. Rectas en imagen original y errores
    def a_imagen_original(self, linea):
        """Proyecta la recta (vista cenital) a la imagen original (escala de proceso)."""
        h = self.h
        # Dos puntos de la recta en la vista cenital
        pts = np.float32([[linea["a"] * 0 + linea["b"], 0],
                          [linea["a"] * h + linea["b"], h]]).reshape(-1, 1, 2)
        orig = cv2.perspectiveTransform(pts, self.M_inv).reshape(-1, 2)
        (x1, y1), (x2, y2) = orig
        dy = y2 - y1 if abs(y2 - y1) > 1e-6 else 1e-6
        a = (x2 - x1) / dy
        b = x1 - a * y1
        return {"a": float(a), "b": float(b), "y_min": float(h * p.Y_SUPERIOR)}

    def calcular_errores(self, izq_o, der_o):
        """izq_o / der_o: rectas x = a*y + b en la imagen ORIGINAL."""
        y_ref = self.h * p.Y_REFERENCIA
        x_izq = izq_o["a"] * y_ref + izq_o["b"]
        x_der = der_o["a"] * y_ref + der_o["b"]
        if x_der <= x_izq:
            return None
        x_centro = 0.5 * (x_izq + x_der)
        semiancho = 0.5 * (x_der - x_izq)
        x_robot = self.w / 2.0

        # Error lateral (0 = centro entre las dos lineas), normalizado: +-1 = sobre una linea
        err_lat = (x_robot - x_centro) / semiancho

        # Error angular (0 = misma pendiente, una "/" y la otra "\")
        # Con x = a*y + b (y hacia abajo): "/" tiene a < 0 y "\" tiene a > 0.
        # Si son simetricas, atan(a_izq) + atan(a_der) = 0.
        ang_izq = np.arctan(izq_o["a"])
        ang_der = np.arctan(der_o["a"])
        err_ang_rad = 0.5 * (ang_izq + ang_der)

        return {
            "error_lateral": float(np.clip(err_lat, -1.0, 1.0)),
            "error_lateral_px": float((x_robot - x_centro) / self.escala),
            "error_angular_rad": float(err_ang_rad),
            "error_angular": float(np.clip(err_ang_rad / np.radians(p.ANGULO_MAX_GRADOS), -1.0, 1.0)),
            "x_izq": float(x_izq), "x_der": float(x_der), "x_centro": float(x_centro),
        }

    # Funcion principal
    def procesar(self, frame_bgr, dibujar=True):
        pequena = cv2.resize(frame_bgr, (self.w, self.h), interpolation=cv2.INTER_AREA)
        mascara = self.mascara_blanco(pequena)
        cenital = cv2.warpPerspective(mascara, self.M, (self.w, self.h), flags=cv2.INTER_NEAREST)

        lineas = self.buscar_lineas(cenital)
        par = self.elegir_par(lineas)

        resultado = {"hay_pareja": False, "lineas_detectadas": len(lineas),
                     "lineas_descartadas": self.descartadas}
        izq_o = der_o = None
        if par is not None:
            izq, der = par
            izq_o = self.a_imagen_original(izq)
            der_o = self.a_imagen_original(der)
            errores = self.calcular_errores(izq_o, der_o)
            if errores is not None:
                resultado.update(errores)
                resultado["hay_pareja"] = True
                self.x_izq_prev, self.x_der_prev = izq["x_ref"], der["x_ref"]
        if not resultado["hay_pareja"]:
            self.x_izq_prev = self.x_der_prev = None

        salida = None
        if dibujar:
            salida = self.dibujar(pequena, mascara, resultado, izq_o, der_o)
        return resultado, salida

    # Visualizacion (solo la imagen con las lineas detectadas)
    def dibujar(self, pequena, mascara, res, izq_o, der_o):
        out = pequena  # se dibuja encima (ya es una copia reducida)
        out[mascara > 0] = (255, 0, 255)  # pixeles detectados como linea
        h, w = self.h, self.w
        cv2.line(out, (w // 2, self.y0), (w // 2, h), (255, 0, 0), 1)
        if res["hay_pareja"]:
            for lin, color in ((izq_o, (0, 255, 0)), (der_o, (0, 255, 255))):
                y1, y2 = int(lin["y_min"]), h - 1
                x1, x2 = int(lin["a"] * y1 + lin["b"]), int(lin["a"] * y2 + lin["b"])
                cv2.line(out, (x1, y1), (x2, y2), color, 2)
            yr = int(h * p.Y_REFERENCIA)
            cv2.circle(out, (int(res["x_centro"]), yr), 4, (0, 0, 255), -1)
            txt = f"lat {res['error_lateral']:+.2f}  ang {np.degrees(res['error_angular_rad']):+.1f}deg"
            cv2.putText(out, txt, (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        else:
            cv2.putText(out, f"sin par ({res['lineas_detectadas']} lineas)", (5, 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)
        return out

# Funciones de recuperacion (del codigo original, aun no se usan en el nodo)
def evaluar_recuperacion(lineas, frame_original):
    """Detecta si aparece una linea casi horizontal al frente y decide hacia donde ir."""
    if not lineas:
        return {"modo": "sin_lineas", "accion": None}
    pendientes = np.array([linea["a"] for linea in lineas])
    pendiente_promedio = float(np.mean(pendientes))
    # La pendiente es dx/dy: valores grandes = lineas mas horizontales.
    if np.max(np.abs(pendientes)) > 2.0:
        hay_pasto, proporcion_verde, _ = hay_verde(frame_original)
        return {"modo": "linea_frontal", "accion": "derecha" if hay_pasto else "izquierda",
                "hay_verde": hay_pasto, "proporcion_verde": proporcion_verde,
                "pendiente_promedio": pendiente_promedio}
    return {"modo": "normal", "accion": None, "pendiente_promedio": pendiente_promedio}


def hay_verde(frame, proporcion_minima=0.08):
    """Retorna (hay_pasto, proporcion_de_pixeles_verdes, mascara)."""
    h, w = frame.shape[:2]
    roi = frame[int(h * 0.35):int(h * 0.75), int(w * 0.20):int(w * 0.80)]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mascara_verde = cv2.inRange(hsv, np.array([35, 40, 40]), np.array([90, 255, 255]))
    proporcion = np.count_nonzero(mascara_verde) / mascara_verde.size
    return proporcion >= proporcion_minima, proporcion, mascara_verde
