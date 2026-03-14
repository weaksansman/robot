import cv2

cfg_path = '/home/pi/DeepHAZMAT/net/yolo.cfg'
weights_path = '/home/pi/DeepHAZMAT/net/yolo.weights'

try:
    net = cv2.dnn.readNetFromDarknet(cfg_path, weights_path)
    print("Model loaded successfully!")
except Exception as e:
    print(f"Error loading model: {e}")