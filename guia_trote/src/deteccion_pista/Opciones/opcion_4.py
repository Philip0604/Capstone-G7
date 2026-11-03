from ultralytics import YOLO

import cv2
import numpy as np
import time
import re
import select
import sys

import shutil
import os

# Cambia esta ruta a donde guardaste tu dataset en Drive
drive_dataset_path = "/content/drive/MyDrive/Capstone/dataset_2"

# Copiar al entorno de Colab para acelerar el entrenamiento
local_path = "/content/dataset"
if os.path.exists(local_path):
    shutil.rmtree(local_path)

shutil.copytree(drive_dataset_path, local_path)

# ==========================================================
# 4) Entrenar YOLOv8
# ==========================================================
from ultralytics import YOLO
from pathlib import Path

path_model = "/content/drive/MyDrive/Capstone/dataset_2/yolov8n.pt"
path_yaml = "/content/drive/MyDrive/Capstone/dataset_2/data_1.yaml"
model = YOLO(path_model)   # pequeño pero rápido

import os

img_path = "/content/dataset/images/train"
lbl_path = "/content/dataset/labels/train"

imgs = sorted([os.path.splitext(f)[0] for f in os.listdir(img_path)])
labels = sorted([os.path.splitext(f)[0] for f in os.listdir(lbl_path)])

missing = [i for i in imgs if i not in labels]

len(imgs), len(labels), missing[:10]

model.train(
    data=path_yaml,
    epochs=80,
    imgsz=640,
    batch=16,
)

# ==========================================================
# 5) Exportar modelo
# ==========================================================

model.export(format="onnx")  # opcional para Raspberry Pi

metrics = model.val ()
metrics.box.map  # map50-95
metrics.box.map50  # map50
metrics.box.map75  # map75
metrics.box.maps  # a list containing mAP50-95 for each category