
"""
Use yolov8 to perform object detection. 
"""
#need to install ultralytics package - pip install ultralytics.
from ultralytics import YOLO
from ultralytics.models.yolo.detect.predict import DetectionPredictor
import cv2

# Load a model
#model = YOLO("yolov8n.yaml")  # build a new model from scratch
model = YOLO("yolov8s.pt")  # load a pretrained model (recommended for training)

# Use the model

results = model.predict(source="0", show=True, conf=0.4, save=True)
print (results)

#condition: do X when detect one or two person?