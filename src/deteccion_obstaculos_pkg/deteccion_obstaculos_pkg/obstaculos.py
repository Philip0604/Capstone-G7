"""
Deteccion de obstaculos con la imagen de profundidad de la RealSense (sin ROS).

Idea
----
La camara esta a ~25 cm del suelo, asi que gran parte de la imagen de
profundidad es PISO a menos de 5 m. Un umbral "profundidad < 5 m" detectaria el
suelo como obstaculo. En vez de eso:

  1. Calibracion: con el piso despejado se ajusta un PLANO a los puntos 3D del
     suelo (RANSAC + minimos cuadrados). De ahi salen la altura de la camara y
     su inclinacion (pitch) y giro (roll). Se puede promediar sobre varios frames
     y se guarda en un archivo JSON.
  2. Deteccion: cada pixel de profundidad se pasa a un punto 3D y se calcula su
     ALTURA SOBRE EL PISO. Es obstaculo si:
        altura_min + tolerancia_por_metro * distancia < altura < altura_max
        distancia_min <= distancia horizontal <= distancia_max (5 m)
     La tolerancia crece con la distancia porque un error pequeno de
     inclinacion hace que el piso lejano "suba" (1 grado a 5 m = 8.7 cm).
  3. Los pixeles de obstaculo se agrupan (componentes conexas) y se descartan
     los grupos pequenos (ruido / "flying pixels" de la RealSense).

Coordenadas
-----------
Marco optico de la camara: x a la derecha, y hacia abajo, z hacia adelante.
Para un pixel (u, v) con profundidad z (metros):
    P = z * ((u - cx) / fx, (v - cy) / fy, 1)
Plano del piso: altura(P) = altura_camara + arriba . P   (arriba = normal unitaria)
"""

import json
import math
import os

import cv2
import numpy as np


class ErrorCalibracion(RuntimeError):
    pass


# ================================================================ calibracion
class Calibracion:
    """Plano del piso visto desde la camara."""

    def __init__(self, arriba, altura, origen="manual", info=None):
        a = np.asarray(arriba, dtype=np.float64)
        self.arriba = a / np.linalg.norm(a)
        self.altura = float(altura)
        self.origen = origen
        self.info = dict(info or {})
        # Ejes horizontales (sobre el piso): adelante = eje optico proyectado al piso
        z = np.array([0.0, 0.0, 1.0])
        f = z - z.dot(self.arriba) * self.arriba
        self.adelante = f / np.linalg.norm(f)
        self.derecha = np.cross(self.adelante, self.arriba)

    @classmethod
    def desde_angulos(cls, altura_m, inclinacion_deg=0.0, roll_deg=0.0):
        """Calibracion manual: camara a 'altura_m', inclinada hacia abajo 'inclinacion_deg'."""
        p, r = math.radians(inclinacion_deg), math.radians(roll_deg)
        arriba = np.array([0.0, -math.cos(p), -math.sin(p)])
        rot_z = np.array([[math.cos(r), -math.sin(r), 0], [math.sin(r), math.cos(r), 0], [0, 0, 1]])
        return cls(rot_z @ arriba, altura_m, origen="manual")

    @property
    def inclinacion_deg(self):
        """> 0: la camara mira hacia abajo."""
        return math.degrees(math.asin(float(np.clip(-self.arriba[2], -1, 1))))

    @property
    def roll_deg(self):
        return math.degrees(math.atan2(self.arriba[0], -self.arriba[1]))

    def a_dict(self):
        return {"arriba": [float(x) for x in self.arriba], "altura_m": self.altura,
                "inclinacion_deg": self.inclinacion_deg, "roll_deg": self.roll_deg,
                "origen": self.origen, "info": self.info}

    @classmethod
    def desde_dict(cls, d):
        return cls(d["arriba"], d["altura_m"], d.get("origen", "archivo"), d.get("info"))

    def guardar(self, ruta):
        ruta = os.path.expanduser(ruta)
        os.makedirs(os.path.dirname(ruta) or ".", exist_ok=True)
        with open(ruta, "w") as f:
            json.dump(self.a_dict(), f, indent=2)

    @classmethod
    def cargar(cls, ruta):
        with open(os.path.expanduser(ruta)) as f:
            return cls.desde_dict(json.load(f))

    def __repr__(self):
        return (f"Calibracion(altura={self.altura:.3f} m, inclinacion={self.inclinacion_deg:+.2f} deg, "
                f"roll={self.roll_deg:+.2f} deg, origen={self.origen})")


class Intrinsecos:
    """Parametros de la camara de profundidad (de CameraInfo o por defecto)."""

    # D435, profundidad 640x480 (valores tipicos; el nodo usa los reales de /camera_info)
    FX_640, FY_640 = 385.0, 385.0

    def __init__(self, fx, fy, cx, cy, ancho, alto):
        self.fx, self.fy, self.cx, self.cy = float(fx), float(fy), float(cx), float(cy)
        self.ancho, self.alto = int(ancho), int(alto)

    @classmethod
    def por_defecto(cls, ancho, alto, fx_640=FX_640, fy_640=FY_640):
        s = ancho / 640.0
        return cls(fx_640 * s, fy_640 * s, (ancho - 1) / 2.0, (alto - 1) / 2.0, ancho, alto)

    def escalar_a(self, ancho, alto):
        if (ancho, alto) == (self.ancho, self.alto):
            return self
        sx, sy = ancho / self.ancho, alto / self.alto
        return Intrinsecos(self.fx * sx, self.fy * sy, self.cx * sx, self.cy * sy, ancho, alto)

    def clave(self):
        return (self.fx, self.fy, self.cx, self.cy, self.ancho, self.alto)


def rayos(intr, paso):
    """Direcciones (x/z, y/z) de los pixeles muestreados cada 'paso'."""
    us = np.arange(0, intr.ancho, paso, dtype=np.float32)
    vs = np.arange(0, intr.alto, paso, dtype=np.float32)
    rx = ((us - intr.cx) / intr.fx)[None, :].repeat(len(vs), 0)
    ry = ((vs - intr.cy) / intr.fy)[:, None].repeat(len(us), 1)
    return rx, ry


def puntos_3d(prof_m, intr, paso, fila_inicio=0.0, z_max=None):
    """Nube de puntos (N x 3, metros) de los pixeles validos."""
    rx, ry = rayos(intr, paso)
    z = prof_m[::paso, ::paso]
    validos = z > 0
    if fila_inicio > 0:
        validos[: int(z.shape[0] * fila_inicio)] = False
    if z_max is not None:
        validos &= z <= z_max
    zz = z[validos]
    return np.stack([rx[validos] * zz, ry[validos] * zz, zz], axis=1).astype(np.float64)


def ajustar_plano_piso(puntos, umbral_m=0.02, iteraciones=300, angulo_max_deg=45.0,
                       semilla=0):
    """
    RANSAC de un plano "horizontal" (normal a menos de angulo_max_deg de -y de la
    camara) + refinamiento por minimos cuadrados.
    Retorna (arriba, altura, fraccion_inliers).
    """
    n_pts = len(puntos)
    if n_pts < 100:
        raise ErrorCalibracion(f"Muy pocos puntos de piso validos ({n_pts}). "
                               "Revisa que la camara vea el suelo.")
    rng = np.random.default_rng(semilla)
    muestra = puntos[rng.choice(n_pts, min(n_pts, 3000), replace=False)]

    idx = rng.integers(0, n_pts, (iteraciones, 3))
    p1, p2, p3 = puntos[idx[:, 0]], puntos[idx[:, 1]], puntos[idx[:, 2]]
    normales = np.cross(p2 - p1, p3 - p1)
    largo = np.linalg.norm(normales, axis=1)
    ok = largo > 1e-9
    normales, p1 = normales[ok] / largo[ok, None], p1[ok]
    normales[normales[:, 1] > 0] *= -1                       # "arriba" = hacia -y
    ok = -normales[:, 1] >= math.cos(math.radians(angulo_max_deg))
    normales, p1 = normales[ok], p1[ok]
    d = -np.einsum("ij,ij->i", normales, p1)                 # altura de la camara
    ok = d > 0
    normales, d = normales[ok], d[ok]
    if len(normales) == 0:
        raise ErrorCalibracion("No se encontro ningun plano horizontal bajo la camara.")

    votos = (np.abs(muestra @ normales.T + d) < umbral_m).sum(axis=0)
    mejor = int(np.argmax(votos))
    n, h = normales[mejor], d[mejor]

    # Refinamiento con todos los inliers (2 pasadas)
    for _ in range(2):
        inl = puntos[np.abs(puntos @ n + h) < umbral_m]
        c = inl.mean(axis=0)
        _, _, vt = np.linalg.svd(inl - c, full_matrices=False)
        n = vt[-1] if vt[-1][1] < 0 else -vt[-1]
        h = -float(n @ c)
    fraccion = float(np.mean(np.abs(puntos @ n + h) < umbral_m))
    return n, h, fraccion


class Calibrador:
    """
    Junta 'n_frames' frames de profundidad del piso despejado y estima el plano.
      cal = Calibrador(intr, n_frames=15)
      for frame in frames: res = cal.agregar(frame)   # None hasta terminar
    Al terminar retorna una Calibracion o lanza ErrorCalibracion.
    """

    def __init__(self, n_frames=15, altura_esperada=0.25, tolerancia_altura=0.15,
                 fila_inicio=0.5, distancia_max=4.0, fraccion_min=0.5, umbral_m=0.02,
                 escala_profundidad=0.001, paso=4):
        self.n_frames = max(1, int(n_frames))
        self.altura_esperada = altura_esperada
        self.tolerancia_altura = tolerancia_altura
        self.fila_inicio, self.distancia_max = fila_inicio, distancia_max
        self.fraccion_min, self.umbral_m = fraccion_min, umbral_m
        self.escala, self.paso = escala_profundidad, paso
        self.resultados = []
        self.errores = []
        self.vistos = 0

    @property
    def terminado(self):
        return self.vistos >= self.n_frames

    def agregar(self, prof, intr):
        prof_m = prof.astype(np.float32) * self.escala
        self.vistos += 1
        try:
            pts = puntos_3d(prof_m, intr, self.paso, self.fila_inicio, self.distancia_max)
            n, h, frac = ajustar_plano_piso(pts, self.umbral_m, semilla=self.vistos)
            if frac < self.fraccion_min:
                raise ErrorCalibracion(
                    f"solo {100 * frac:.0f}% de la parte baja de la imagen es piso plano "
                    "(hay objetos delante?)")
            self.resultados.append((n, h, frac))
        except ErrorCalibracion as e:
            self.errores.append(str(e))
        if self.terminado:
            return self.resultado()
        return None

    def resultado(self):
        if len(self.resultados) < max(1, self.n_frames // 2):
            motivo = self.errores[-1] if self.errores else "sin frames"
            raise ErrorCalibracion(
                f"Calibracion fallida: {len(self.resultados)}/{self.vistos} frames validos ({motivo})")
        normales = np.array([r[0] for r in self.resultados])
        alturas = np.array([r[1] for r in self.resultados])
        arriba = np.median(normales, axis=0)
        altura = float(np.median(alturas))
        if self.tolerancia_altura and abs(altura - self.altura_esperada) > self.tolerancia_altura:
            raise ErrorCalibracion(
                f"Altura estimada {altura:.3f} m muy distinta de la esperada "
                f"{self.altura_esperada:.2f} m: el plano encontrado no parece ser el piso.")
        info = {"frames": self.vistos, "frames_validos": len(self.resultados),
                "dispersion_altura_m": float(alturas.std()),
                "fraccion_piso": float(np.mean([r[2] for r in self.resultados]))}
        return Calibracion(arriba, altura, origen="automatica", info=info)


# ================================================================== deteccion
class DetectorObstaculos:
    def __init__(self, calibracion, distancia_max=5.0, distancia_min=0.3, altura_min=0.10,
                 altura_max=1.8, tolerancia_por_metro=0.02, ancho_corredor=0.0, paso=4,
                 min_celdas=8, escala_profundidad=0.001):
        self._cache = None
        self.calibracion = calibracion
        self.distancia_max, self.distancia_min = distancia_max, distancia_min
        self.altura_min, self.altura_max = altura_min, altura_max
        self.tol_m = tolerancia_por_metro
        self.ancho_corredor = ancho_corredor
        self.paso = max(1, int(paso))
        self.min_celdas = min_celdas
        self.escala = escala_profundidad
        self._cache = None  # (clave, ru, rf, rr)

    @property
    def calibracion(self):
        return self._calibracion

    @calibracion.setter
    def calibracion(self, cal):
        self._calibracion = cal
        self._cache = None  # los rayos dependen del plano del piso

    def _rayos(self, intr):
        cal = self.calibracion
        clave = (intr.clave(), self.paso)
        if self._cache is None or self._cache[0] != clave:
            rx, ry = rayos(intr, self.paso)
            # productos punto de cada rayo (x/z, y/z, 1) con los ejes del piso
            ru = cal.arriba[0] * rx + cal.arriba[1] * ry + cal.arriba[2]
            rf = cal.adelante[0] * rx + cal.adelante[1] * ry + cal.adelante[2]
            rr = cal.derecha[0] * rx + cal.derecha[1] * ry + cal.derecha[2]
            self._cache = (clave, ru, rf, rr)
        return self._cache[1:]

    def procesar(self, prof, intr):
        """
        prof: imagen de profundidad (uint16 en mm de la RealSense, o float en metros
              con escala_profundidad=1). intr: Intrinsecos.
        Retorna dict con hay_obstaculo, distancia (m), adelante, lateral (+ = derecha),
        altura (m sobre el piso), obstaculos (lista, del mas cercano al mas lejano)
        y mascara (uint8, resolucion reducida por 'paso').
        """
        intr = intr.escalar_a(prof.shape[1], prof.shape[0])
        ru, rf, rr = self._rayos(intr)
        z = prof[::self.paso, ::self.paso].astype(np.float32) * self.escala

        altura = self.calibracion.altura + z * ru
        adelante = z * rf
        lateral = z * rr
        dist = np.hypot(adelante, lateral)

        m = (z > 0) & (adelante > 0) & (dist >= self.distancia_min) & (dist <= self.distancia_max)
        m &= (altura > self.altura_min + self.tol_m * dist) & (altura < self.altura_max)
        if self.ancho_corredor > 0:
            m &= np.abs(lateral) <= self.ancho_corredor / 2.0
        mascara = m.astype(np.uint8)

        n, etiquetas, stats, _ = cv2.connectedComponentsWithStats(mascara, connectivity=8)
        obstaculos = []
        for i in range(1, n):
            area = int(stats[i, cv2.CC_STAT_AREA])
            if area < self.min_celdas:
                mascara[etiquetas == i] = 0
                continue
            sel = etiquetas == i
            d = dist[sel]
            d_cerca = float(np.percentile(d, 10))   # robusto a pixeles sueltos
            cerca = d <= d_cerca + 0.05
            x, y, w, h = (int(v) * self.paso for v in stats[i, :4])
            obstaculos.append({
                "distancia": d_cerca,
                "adelante": float(np.median(adelante[sel][cerca])),
                "lateral": float(np.median(lateral[sel][cerca])),
                "altura": float(np.percentile(altura[sel], 95)),
                "celdas": area,
                "caja_px": (x, y, w, h),
            })
        obstaculos.sort(key=lambda o: o["distancia"])

        res = {"hay_obstaculo": bool(obstaculos), "obstaculos": obstaculos, "mascara": mascara}
        if obstaculos:
            res.update({k: obstaculos[0][k] for k in ("distancia", "adelante", "lateral", "altura")})
        return res
