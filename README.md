# Tracking

This folder contains all relevant files for training and deploying a logo tracking system for the gesture-controlled vehicle project.

## Contents

- **UR_Label_Detection.v1i.yolov8.zip**  
  Custom-collected dataset and labels used for training the YOLOv8n model.

- **best.onnx**  
  Exported ONNX model for logo detection, converted from YOLOv8n.

- **jetson_PID.py**  
  PID control algorithm for the vehicle, based on tracking information.

- **testcamera_output.py**  
  Script to test the RealSense depth camera and print out distance and calculated speed.

- **train.py**  
  YOLOv8 training configuration script using the custom dataset.

## Folder

- **UR_logo_project/**  
  Project folder containing training results and related outputs.

---

Developed as part of the AutoTracking-HandGesture_Controlled_Vehicle system.
