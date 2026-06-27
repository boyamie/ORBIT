    def detect_target(self, frame):
        results = self.model(frame)

        target_info = None
        processed_frame = frame.copy()

        max_area = 0
        best_box = None  # 가장 큰 사람의 좌표를 임시로 저장할 변수

        for result in results:
            for box in result.boxes:
                # 0번 클래스(사람)가 아니면 건너뛰기
                if int(box.cls[0]) != 0:
                    continue

                x1, y1, x2, y2 = map(int, box.xyxy[0])
                w = x2 - x1
                h = y2 - y1
                area = w * h

                # 가장 큰 사람 찾기
                if area > max_area:
                    max_area = area
                    best_box = (x1, y1, x2, y2) # 좌표만 임시 저장해 두기
                    
                    cx = x1 + w // 2
                    cy = y1 + h // 2
                    target_info = (cx, cy, area)

        # 반복문이 완전히 끝난 후, 최종 타겟을 찾았다면 그때 딱 한 번만 그리기
        if best_box is not None:
            x1, y1, x2, y2 = best_box
            cx, cy, _ = target_info
            
            cv2.rectangle(processed_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(processed_frame, (cx, cy), 5, (0, 0, 255), -1)

        return target_info, processed_frame