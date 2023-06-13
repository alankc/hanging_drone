from ultralytics import YOLO

# Load a model
model = YOLO("yolov8x.yaml")  # load a pretrained model (recommended for training)YOLO("yolov8n.yaml")

# Use the model
results = model.train(data="/home/x/Documents/landing_pipeline/yolo_dataset/indoor/train.yaml", epochs=500)  # train the model