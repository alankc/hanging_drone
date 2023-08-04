from ultralytics import YOLO
#args https://docs.ultralytics.com/modes/train/#arguments

train_path = "/home/x/Documents/landing_pipeline/yolo_dataset/natural_branch/train.yaml"


model = YOLO("yolov8x.pt")  # load a pretrained model (recommended for training)YOLO("yolov8n.yaml")
# Use the model
results = model.train(data=train_path, epochs=5000, patience=500)  # train the model