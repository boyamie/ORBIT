import cv2
import time

# 학생들의 모듈 불러오기
from config import TRACKING_CONFIG
from drone_controller import DroneController
from yolo_detector import YoloDetector
from pid import PIDController

def main():
    print("🚀 ORBIT 시스템 통합 테스트를 시작합니다...")

    # 1. 각 모듈 객체 생성
    drone = DroneController()
    detector = YoloDetector(model_path='yolov8n.pt') # 같은 폴더에 yolov8n.pt 파일이 있어야 합니다.

    # 2. 기태 학생의 config 구조에 맞춘 PID 제어기 생성
    pid_yaw = PIDController(TRACKING_CONFIG['yaw']['kp'], TRACKING_CONFIG['yaw']['ki'], TRACKING_CONFIG['yaw']['kd'])
    pid_updown = PIDController(TRACKING_CONFIG['updown']['kp'], TRACKING_CONFIG['updown']['ki'], TRACKING_CONFIG['updown']['kd'])
    pid_forward = PIDController(TRACKING_CONFIG['forward']['kp'], TRACKING_CONFIG['forward']['ki'], TRACKING_CONFIG['forward']['kd'])

    # 3. 드론 연결 및 이륙 (운택 파트)
    if not drone.connect():
        print("❌ 드론 연결 실패. 프로그램을 종료합니다.")
        return

    drone.start_stream()
    time.sleep(2) # 영상 스트리밍 안정화 대기
    
    drone.takeoff()
    time.sleep(2) # 이륙 후 호버링 안정화 대기

    print("✅ 이륙 완료! 카메라에서 사람을 찾습니다. (종료하려면 화면을 클릭하고 'q' 입력)")

    try:
        while True:
            # 4. 카메라 프레임 가져오기
            frame = drone.get_frame()
            if frame is None:
                continue

            # 5. YOLO로 타겟(사람) 찾기 (동권 파트)
            target_info, processed_frame = detector.detect_target(frame)

            # 6. PID 제어 및 드론 이동 (기태 & 운택 파트 통합)
            if target_info:
                cx, cy, area = target_info
                
                # 화면 중심점 구하기
                frame_height, frame_width = processed_frame.shape[:2]
                center_x, center_y = frame_width // 2, frame_height // 2

                # 오차(Error) 계산
                error_x = cx - center_x           # 좌우 오차
                error_y = center_y - cy           # 상하 오차 (y축은 위가 0이므로 반대 방향)
                error_area = TRACKING_CONFIG['target_area'] - area # 거리 오차

                # PID 알고리즘을 통한 이동 속도 도출 (정수형으로 변환)
                yaw_speed = int(pid_yaw.calculate(error_x))
                updown_speed = int(pid_updown.calculate(error_y))
                forward_speed = int(pid_forward.calculate(error_area))

                # 드론에 이동 명령 전송: send_command(좌우, 전후진, 상하, 회전)
                # 좌우 이동(Roll)은 0으로 고정하고 회전(Yaw)으로 추적합니다.
                drone.send_command(0, forward_speed, updown_speed, yaw_speed)
                
                # 터미널 대신 화면에 현재 제어값 출력 (디버깅용)
                cv2.putText(processed_frame, f"Tracking: FWD({forward_speed}) UP({updown_speed}) YAW({yaw_speed})", 
                            (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            else:
                # 사람을 놓치면 속도를 0으로 만들어 제자리 비행(Hovering)
                drone.send_command(0, 0, 0, 0)
                cv2.putText(processed_frame, "Target Lost - Hovering", 
                            (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            # 결과 화면 보여주기
            cv2.imshow("ORBIT Vision", processed_frame)

            # 'q' 키를 누르면 루프 탈출
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except Exception as e:
        print(f"⚠️ 시스템 구동 중 오류 발생: {e}")

    finally:
        # 7. 안전 종료 (가장 중요)
        print("🛬 시스템을 종료하고 드론을 안전하게 착륙시킵니다.")
        drone.send_command(0, 0, 0, 0) # 모든 모터 정지 명령
        drone.land()
        drone.stop_stream()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()