import cv2
import time

# 1. 학생들이 작성할 모듈 불러오기 (Import)
from config import TRACKING_CONFIG                 # 기태 파트: 설정값
from drone_controller import DroneController       # 운택 파트: 드론 제어
from yolo_detector import YoloDetector             # 동권 파트: YOLO 비전
from pid import PIDController                      # 기태 파트: PID 알고리즘

def main():
    print("🚀 ORBIT 시스템 초기화를 시작합니다...")

    # 2. 각 모듈 객체 생성 (Initialization)
    drone = DroneController()
    detector = YoloDetector(model_path='yolov8n.pt')
    
    # PID 제어기 3개 생성 (상하, 좌우 회전, 전후진)
    pid_yaw = PIDController(TRACKING_CONFIG['yaw']['kp'], TRACKING_CONFIG['yaw']['ki'], TRACKING_CONFIG['yaw']['kd'])
    pid_updown = PIDController(TRACKING_CONFIG['Kp_updown'], TRACKING_CONFIG['Ki_updown'], TRACKING_CONFIG['Kd_updown'])
    pid_forward = PIDController(TRACKING_CONFIG['Kp_forward'], TRACKING_CONFIG['Ki_forward'], TRACKING_CONFIG['Kd_forward'])

    # 3. 드론 연결 및 이륙
    if not drone.connect():
        print("❌ 드론 연결에 실패했습니다. 프로그램을 종료합니다.")
        return

    drone.start_stream()
    drone.takeoff()
    time.sleep(2) # 이륙 후 안정화 대기

    print("✅ 이륙 완료! 자율 추적을 시작합니다. (종료하려면 'q' 입력)")

    # 4. 메인 추적 루프 (Main Loop)
    try:
        while True:
            # 4-1. 카메라 프레임 가져오기 (운택 파트)
            frame = drone.get_frame()
            if frame is None:
                continue

            # 4-2. YOLO로 타겟 찾기 (동권 파트)
            # 타겟의 중심점(cx, cy)과 크기(area)를 반환받음
            target_info, processed_frame = detector.detect_target(frame)

            # 4-3. 타겟이 화면에 있을 경우 PID 제어 수행 (기태 파트)
            if target_info:
                cx, cy, area = target_info
                
                # 화면 중앙 좌표 계산
                frame_height, frame_width = processed_frame.shape[:2]
                center_x, center_y = frame_width // 2, frame_height // 2

                # 오차(Error) 계산
                error_x = cx - center_x  # 좌우 오차
                error_y = center_y - cy  # 상하 오차 (y축은 위로 갈수록 값이 작아지므로 반대로 뺌)
                error_area = TRACKING_CONFIG['target_area'] - area # 거리(크기) 오차

                # PID 알고리즘으로 이동 속도 계산
                yaw_speed = pid_yaw.calculate(error_x)
                updown_speed = pid_updown.calculate(error_y)
                forward_speed = pid_forward.calculate(error_area)

                # 드론에게 이동 명령 전송 (운택 파트)
                # send_rc_control(좌우, 전후진, 상하, 회전)
                drone.send_command(0, forward_speed, updown_speed, yaw_speed)

            else:
                # 타겟을 잃어버리면 제자리 비행 (Hovering)
                drone.send_command(0, 0, 0, 0)

            # 4-4. 화면 출력
            cv2.imshow("ORBIT Tracking Viewer", processed_frame)

            # 'q' 키를 누르면 루프 종료
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except Exception as e:
        print(f"⚠️ 실행 중 오류 발생: {e}")

    finally:
        # 5. 안전 종료 (착륙 및 자원 해제)
        print("🛬 드론을 안전하게 착륙시키고 시스템을 종료합니다.")
        drone.land()
        drone.stop_stream()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()