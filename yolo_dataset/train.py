from ultralytics import YOLO
#args https://docs.ultralytics.com/modes/train/#arguments
# Load a model
model = YOLO("yolov8x.pt")  # load a pretrained model (recommended for training)YOLO("yolov8n.yaml")

# Use the model
results = model.train(data="/home/x/Documents/landing_pipeline/yolo_dataset/mixed/train.yaml", epochs=100, patience=100, batch=-1)  # train the model