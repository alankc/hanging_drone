from ultralytics import YOLO
#args https://docs.ultralytics.com/modes/train/#arguments

train_path = "/home/x/Documents/landing_pipeline/yolo_dataset/indoor/train.yaml"

# Load a model --------> train 9
model = YOLO("yolov8l.pt")  # load a pretrained model (recommended for training)YOLO("yolov8n.yaml")
# Use the model
results = model.train(data=train_path, epochs=1000, patience=200)  # train the model