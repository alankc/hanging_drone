from ultralytics import YOLO

# Load a model
model = YOLO("best.pt")  # load a pretrained model (recommended for training)

# Use the model
results = model.train(data="train.yaml", epochs=500)  # train the model