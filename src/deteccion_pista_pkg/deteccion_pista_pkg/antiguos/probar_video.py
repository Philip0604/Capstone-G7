"""
Prueba el detector de pista con un video, SIN ROS ni camara (util en el PC).

Uso:
  ros2 run deteccion_pista_pkg probar_video videos/input/Pista_zigzag.mp4
  python3 -m deteccion_pista_pkg.probar_video videos/input/Pista_zigzag.mp4

Teclas: espacio = pausa, q / ESC = salir.
"""
import sys
import time

import cv2

from deteccion_pista_pkg.deteccion import DetectorPista


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return
    cap = cv2.VideoCapture(sys.argv[1])
    det = None
    pausa = False
    while True:
        if not pausa:
            ok, frame = cap.read()
            if not ok:
                break
            # Simula la resolucion de la camara del robot (ajustar si cambia)
            frame = cv2.resize(frame, (640, 480))
            if det is None:
                det = DetectorPista(frame.shape[1], frame.shape[0])
            t0 = time.perf_counter()
            res, salida = det.procesar(frame)
            dt = (time.perf_counter() - t0) * 1000
            if res["hay_pareja"]:
                print(f"lat {res['error_lateral']:+.2f}  ang {res['error_angular']:+.2f}  ({dt:.1f} ms)")
            else:
                print(f"sin par ({dt:.1f} ms)")
            cv2.imshow("Pista - lineas detectadas", cv2.resize(salida, None, fx=2, fy=2))
        k = cv2.waitKey(30) & 0xFF
        if k in (ord("q"), 27):
            break
        if k == ord(" "):
            pausa = not pausa
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
