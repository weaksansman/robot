import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2
import numpy as np
import os

class YOLOHazmatDetector(Node):
    def __init__(self):
        super().__init__('yolo_hazmat4')
        self.bridge = CvBridge()

        # เปิดกล้องโดยตรง
        self.cap = cv2.VideoCapture(2)  # เปิดกล้อง webcam (เปลี่ยนเลขเป็น 1, 2 ถ้ามีหลายตัว)

        if not self.cap.isOpened():
            self.get_logger().error("Cannot open camera")
            return

        # กำหนด Path ของไฟล์
        cfg_path = '~/ros2_ws/src/robot/datahazmat/netyolo.cfg'
        weights_path = '~/ros2_ws/src/robot/datahazmat/net/yolo.weights'
        names_path = '~/ros2_ws/src/robot/datahazmat/net/labels.names'

        # ตรวจสอบไฟล์โมเดล
        if not os.path.exists(cfg_path) or not os.path.exists(weights_path) or not os.path.exists(names_path):
            self.get_logger().error("Model files not found!")
            return

        # โหลดโมเดล YOLO
        self.net = cv2.dnn.readNetFromDarknet(cfg_path, weights_path)

        # โหลด Labels
        with open(names_path, 'r') as f:
            self.classes = [line.strip() for line in f.readlines()]

        # กำหนด Layer Names
        self.layer_names = self.net.getLayerNames()
        unconnected_layers = self.net.getUnconnectedOutLayers().flatten()  # แก้ไขการดึง layer
        self.output_layers = [self.layer_names[i - 1] for i in unconnected_layers]

        self.get_logger().info("YOLO Hazmat Detector Ready!")

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to grab frame from camera")
            return

        # แปลงภาพให้เหมาะกับ YOLO
        blob = cv2.dnn.blobFromImage(frame, 1/255.0, (416, 416), swapRB=True, crop=False)
        self.net.setInput(blob)
        outs = self.net.forward(self.output_layers)

        # ตรวจจับวัตถุ
        class_ids = []
        confidences = []
        boxes = []

        height, width = frame.shape[:2]
        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > 0.5:
                    box = detection[:4] * np.array([width, height, width, height])
                    (centerX, centerY, w, h) = box.astype("int")

                    x = int(centerX - (w / 2))
                    y = int(centerY - (h / 2))

                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)

        indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)

        # วาดกรอบสี่เหลี่ยม
        for i in range(len(boxes)):
            if i in indexes:
                x, y, w, h = boxes[i]
                label = f"{self.classes[class_ids[i]]}: {confidences[i]:.2f}"
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(frame, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # แสดงผลภาพ
        cv2.imshow("Hazmat Detection", frame)
        cv2.waitKey(1)

    def run(self):
        while rclpy.ok():
            self.process_frame()

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = YOLOHazmatDetector()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
