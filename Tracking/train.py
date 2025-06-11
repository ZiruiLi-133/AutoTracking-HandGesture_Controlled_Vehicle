import os
os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"

from ultralytics import YOLO

# load the model（also can use 'yolov8m.pt' or 'yolov8l.pt'）
model = YOLO('yolov8n.pt')  # n=Nano（lighter）

# training
results = model.train(
    data='UR_Label_Detection.v1i.yolov8/data.yaml',  
    epochs=30,                         
    imgsz=640,                         
    batch=8,                           
    project='UR_logo_project',         
    name='yolov8n-rochester-logo',     
    pretrained=True                  
)
