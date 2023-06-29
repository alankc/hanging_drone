from ultralytics import YOLO
#args https://docs.ultralytics.com/modes/train/#arguments
# Load a model
model = YOLO("yolov8n.pt")  # load a pretrained model (recommended for training)YOLO("yolov8n.yaml")

# Use the model
results = model.train(data="/home/alan/Documentos/landing_pipeline/yolo_dataset/indoor/train.yaml", epochs=1000, patience=200)  # train the model