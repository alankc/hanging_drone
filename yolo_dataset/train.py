from ultralytics import YOLO
#args https://docs.ultralytics.com/modes/train/#arguments
# Load a model
model = YOLO("yolov8x.yaml")  # load a pretrained model (recommended for training)YOLO("yolov8n.yaml")

# Use the model
results = model.train(data="/home/x/Documents/landing_pipeline/yolo_dataset/indoor/train.yaml", epochs=500, patience=100, lr0=0.1, lrf=0.001)  # train the model