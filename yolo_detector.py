from ultralytics import YOLO
import cv2

class YoloDetector:
    def __init__(self, model_path="models/yolov8n.pt"):
        self.model = YOLO(model_path)

    def detect_target(self, frame):
        results = self.model(frame)

        target_info = None
        processed_frame = frame.copy()

        max_area = 0

        for result in results:
            for box in result.boxes:

                if int(box.cls[0]) != 0:
                    continue

                x1, y1, x2, y2 = map(int, box.xyxy[0])

                w = x2 - x1
                h = y2 - y1
                area = w * h

                if area > max_area:
                    max_area = area

                    cx = x1 + w // 2
                    cy = y1 + h // 2

                    target_info = (cx, cy, area)

                    cv2.rectangle(processed_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.circle(processed_frame, (cx, cy), 5, (0, 0, 255), -1)

        return target_info, processed_frame