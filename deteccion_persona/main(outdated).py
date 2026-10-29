import cv2
import numpy as np
import time

fps_detection = 4  # de supuestos; se puede aumentar
frame_detection_time = 1 / fps_detection

camera_height = 35  # in cm
avg_head_height = 22

height_list = list()

detector = cv2.FaceDetectorYN.create("face_detection_yunet_2026may.onnx", "", (480, 640))
detector.setInputSize((480, 640))
cap = cv2.VideoCapture(0)

def calibrate_distance(box_height, measured_distance):
    height_list.append(box_height)
    if len(height_list) > 15:
        del height_list[0]
    height_average = sum(height_list) / 15

    print(height_average * measured_distance, avg_head_height / height_average)  # diag constant, height constant

def distance_calculation(diag_const, height_const, box_height): # distance in cm
    diag_distance = diag_const / box_height
    head_height = box_height * height_const

    #return np.sqrt(diag_distance**2 - (7.5*head_height - camera_height)**2)
    return diag_distance


# main loop:
while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break
        
    h, w, _ = frame.shape
    detector.setInputSize((w, h))
    
    retval, faces = detector.detect(frame)
    
    if faces is not None:
        for face in faces:
            box = face[0:4].astype(np.int32)

            #calibrate_distance(box[3], 85)
            distance = distance_calculation(167, 1.671, box[3])
            print(distance)

            cv2.rectangle(frame, (box[0], box[1]), (box[0] + box[2], box[1] + box[3]), (255, 0, 0), 2)
            
            landmarks = face[4:14].reshape((5, 2)).astype(np.int32)
            for lm in landmarks:
                cv2.circle(frame, tuple(lm), 2, (0, 255, 255), -1)
                
    cv2.imshow('YuNet Real-Time Tracking', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    time.sleep(frame_detection_time)

cap.release()
cv2.destroyAllWindows()
