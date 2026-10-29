import cv2
import numpy as np
import time
import serial

# constants 
fps_detection = 10
frame_detection_time = 1 / fps_detection
model_path = "face_detection_yunet_2026may.onnx"
camera_index = 1  # en computador: 0->camara laptop, 1->camara externa
frame_width = 640  # RECALIBRAR RANGO DISTANCIA SI SE CAMBIA
frame_height = 360 # RECALIBRAR RANGO DISTANCIA SI SE CAMBIA

arduino_port = "/dev/ttyACM0"
baud_rate = 9600

# variables
width_avg = 0
no_face_timer = 0
elapsed = 0
face_detected = False

faces_widths = list()
near_widths = list()

# ----------------- begin code ----------------------------------------------- #
ser = serial.Serial(arduino_port, baud_rate)
time.sleep(2)  # espera a que se conecte correctamente

cap = cv2.VideoCapture(camera_index, cv2.CAP_DSHOW)  # DirectShow para iniciar mas rapido (cv2.CAP_V4L2 para linux)

# se limitan las caracteristicas de la camara para no usar tantos recursos
cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)
cap.set(cv2.CAP_PROP_AUTOFOCUS, 0) # deshabilitar autofocus (deberia mejorar un poco el rendimiento)
cap.set(cv2.CAP_PROP_FPS, 10)  # tal vez cambiar (para evitar frame lag)

# se lee las dimensiones verdaderas (por si no se actualizaron correctamente los limites)
w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

detector = cv2.FaceDetectorYN.create(model_path, "", (w, h))

# main loop:
while cap.isOpened():
    loop_start = time.monotonic()

    ret, frame = cap.read()
    if not ret:
        print("Cant read image") # por si no puede tomar un frame
        break
    
    _, faces = detector.detect(frame)  # _ = retval (no se usa)
    
    if faces is not None:
        for face in faces:
            box = face[0:4].astype(np.int32)
            faces_widths.append(box[2])

    nearest_face_width = max(faces_widths, default=None)  # detecta a persona mas cercana a camara
    faces_widths = list()

    if nearest_face_width == None:
        if face_detected == True:
            no_face_timer = time.monotonic()
        face_detected = False
    else:
        face_detected = True
        near_widths.append(nearest_face_width)

        # promedio de ancho ultimos 10 frames
        if len(near_widths) == 10:
            width_avg = int(sum(near_widths) / 10)
            del near_widths[0]
    
    if not face_detected:
        elapsed = time.monotonic() - no_face_timer
        if elapsed > 3:
            # se envia señal de persona no encontrada a camara
            print("no face detected")

    # para enviar integers al arduino
    message = (str(width_avg) + '\n').encode('utf-8')  # la referencia deberia ser 35, se admite error de 33 a 37 (+-20cm)
    ser.write(message)


    elapsed = time.monotonic() - loop_start
    sleep_time = frame_detection_time - elapsed

    if sleep_time > 0:
        time.sleep(sleep_time)  # para que el programa funcione con 10fps
    else:
        time.sleep(frame_detection_time)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):   # stop if q is pressed (cambiarlo por boton apagado o otra cosa)
        break

else:
    print("Camera disconnected")


cap.release()
cv2.destroyAllWindows()
ser.close()


#idea (from AI):
# To achieve a hard 10 FPS cap without causing video lag, do not use time.sleep(0.1) 
# inside your camera loop. Instead, let your camera pull frames at its natural speed (e.g., 30 FPS) 
# and use a simple modulo counter to only pass every 3rd frame to cv.FaceDetectorYN.