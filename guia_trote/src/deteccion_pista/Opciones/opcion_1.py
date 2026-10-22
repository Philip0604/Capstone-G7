import cv2
import torch
import ultralytics

print("Python funcionando")
print("PyTorch:", torch.__version__)
print("OpenCV:", cv2.__version__)
print("Ultralytics:", ultralytics.__version__)
print("CUDA disponible:", torch.cuda.is_available())

# VIDEO = "videos/Pista_zigzag.mp4"


# cap = cv2.VideoCapture(VIDEO)

# if not cap.isOpened():
#     print("Error: no se pudo abrir el video")
#     exit()


# while True:

#     ret, frame = cap.read()

#     if not ret:
#         break

#     cv2.imshow("Pista", frame)

#     tecla = cv2.waitKey(1) & 0xFF

#     if tecla == ord("q"):
#         break


# cap.release()
# cv2.destroyAllWindows()