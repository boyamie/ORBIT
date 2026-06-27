"""ORBIT autonomous tracking entrypoint.

Connects to a DJI Tello drone, runs YOLOv8 person detection on live frames,
and uses PID control to keep the target centered.
"""

from __future__ import annotations

import sys
from typing import Optional, Tuple

import cv2
from djitellopy import Tello
from simple_pid import PID
from ultralytics import YOLO

from config import (
    FRAME_HEIGHT,
    FRAME_WIDTH,
    MAX_VELOCITY,
    PERSON_CLASS_ID,
    PID_X,
    PID_Y,
    PID_Z,
    TARGET_AREA_RATIO,
    YOLO_WEIGHTS,
)

BoundingBox = Tuple[int, int, int, int]


def clamp(value: float, minimum: int, maximum: int) -> int:
    """Clamp a numeric control output to the allowed drone velocity range."""
    return int(max(minimum, min(maximum, value)))


def connect_drone() -> Tello:
    """Connect to Tello and start video stream with error handling."""
    drone = Tello()
    try:
        drone.connect()
        drone.streamon()
    except Exception as exc:  # pragma: no cover - hardware/network dependent
        raise RuntimeError(f"Failed to connect to Tello or start stream: {exc}") from exc

    print(f"Connected to Tello. Battery: {drone.get_battery()}%")
    return drone


def initialize_pids(target_area: float) -> Tuple[PID, PID, PID]:
    """Create and configure PID controllers for x, y, and z axes."""
    pid_x = PID(PID_X["kp"], PID_X["ki"], PID_X["kd"], setpoint=0)
    pid_y = PID(PID_Y["kp"], PID_Y["ki"], PID_Y["kd"], setpoint=0)
    pid_z = PID(PID_Z["kp"], PID_Z["ki"], PID_Z["kd"], setpoint=target_area)

    for pid in (pid_x, pid_y, pid_z):
        pid.sample_time = 0.0
        pid.output_limits = (-MAX_VELOCITY, MAX_VELOCITY)

    return pid_x, pid_y, pid_z


def find_largest_person(result) -> Optional[BoundingBox]:
    """Return the largest detected person bounding box from a YOLO result."""
    boxes = getattr(result, "boxes", None)
    if boxes is None:
        return None

    best_box: Optional[BoundingBox] = None
    best_area = 0

    for box in boxes:
        class_id = int(box.cls[0])
        if class_id != PERSON_CLASS_ID:
            continue

        x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
        area = max(0, x2 - x1) * max(0, y2 - y1)

        if area > best_area:
            best_area = area
            best_box = (x1, y1, x2, y2)

    return best_box


def run_tracking() -> None:
    """Main tracking loop."""
    model = YOLO(YOLO_WEIGHTS)

    drone = connect_drone()
    frame_reader = drone.get_frame_read()

    frame_area = FRAME_WIDTH * FRAME_HEIGHT
    target_area = TARGET_AREA_RATIO * frame_area
    pid_x, pid_y, pid_z = initialize_pids(target_area)

    print("Press 'q' to stop tracking.")

    try:
        while True:
            frame = frame_reader.frame
            if frame is None:
                continue

            frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))
            result = model(frame, verbose=False)[0]
            person_box = find_largest_person(result)

            left_right = up_down = forward_back = yaw = 0

            if person_box is not None:
                x1, y1, x2, y2 = person_box
                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2
                box_area = (x2 - x1) * (y2 - y1)

                x_error = cx - (FRAME_WIDTH // 2)
                y_error = cy - (FRAME_HEIGHT // 2)

                # Invert y command: positive y_error means target is lower in frame.
                left_right = clamp(pid_x(-x_error), -MAX_VELOCITY, MAX_VELOCITY)
                up_down = clamp(-pid_y(-y_error), -MAX_VELOCITY, MAX_VELOCITY)
                forward_back = clamp(pid_z(box_area), -MAX_VELOCITY, MAX_VELOCITY)

                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)

            drone.send_rc_control(left_right, forward_back, up_down, yaw)
            cv2.imshow("ORBIT Tracking", frame)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    except KeyboardInterrupt:
        print("\nTracking interrupted by user.")
    except Exception as exc:
        print(f"Tracking error: {exc}", file=sys.stderr)
    finally:
        drone.send_rc_control(0, 0, 0, 0)
        drone.streamoff()
        drone.end()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    run_tracking()
