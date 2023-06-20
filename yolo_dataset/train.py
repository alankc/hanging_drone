from ultralytics import YOLO
#args https://docs.ultralytics.com/modes/train/#arguments
# Load a model
model = YOLO("yolov8x.pt")  # load a pretrained model (recommended for training)YOLO("yolov8n.yaml")

# Use the model
results = model.train(data="/home/x/Documents/landing_pipeline/yolo_dataset/mixed/train.yaml", epochs=1000, patience=200)  # train the model