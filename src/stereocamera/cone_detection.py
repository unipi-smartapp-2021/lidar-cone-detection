import torch

# Model
model = torch.hub.load('ultralytics/yolov5', 'custom', path='src/stereocamera/best.pt')

# Image
img = 'src/datasets/example_dataset/images/example_image.jpg'

# Inference
results = model(img)
results.show()
