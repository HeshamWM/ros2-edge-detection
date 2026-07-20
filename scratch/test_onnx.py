import cv2
import numpy as np

# Load model
net = cv2.dnn.readNet("src/pi_yolov26_detector/models/yolo26n.onnx")

# Check input and output details
# Preprocess
dummy_img = np.zeros((640, 640, 3), dtype=np.uint8)
blob = cv2.dnn.blobFromImage(dummy_img, 1.0 / 255.0, (640, 640), (0, 0, 0), swapRB=True, crop=False)
net.setInput(blob)

# Forward pass
outputs = net.forward(net.getUnconnectedOutLayersNames())
print("Number of output layers:", len(outputs))
for i, out in enumerate(outputs):
    print(f"Output {i} shape:", out.shape)
    # Let's inspect some of the values
    print("Sample slice from output:")
    # out has shape (1, 84, 8400)
    print("Bounding box coords slice (first 5 boxes):")
    print(out[0, 0:4, 0:5])
    print("Class scores slice (first 5 classes, first 5 boxes):")
    print(out[0, 4:9, 0:5])
