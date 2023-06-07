from ultralytics import YOLO

# Load a model
model = YOLO("yolov8x.pt")  # load a pretrained model (recommended for training)

# Use the model
results = model.train(data="/home/x/Documents/landing_pipeline/yolo_dataset/indoor/train.yaml", epochs=500)  # train the model