"""Configuration values for ORBIT autonomous drone tracking."""

# YOLO model configuration
YOLO_WEIGHTS = "yolov8n.pt"
PERSON_CLASS_ID = 0

# Video frame dimensions used for control calculations
FRAME_WIDTH = 960
FRAME_HEIGHT = 720

# Target area ratio (person bbox area / frame area) used for distance control
TARGET_AREA_RATIO = 0.14

# PID gains for horizontal (x), vertical (y), and depth (z) control loops
PID_X = {"kp": 0.12, "ki": 0.0, "kd": 0.05}
PID_Y = {"kp": 0.12, "ki": 0.0, "kd": 0.05}
PID_Z = {"kp": 0.00008, "ki": 0.0, "kd": 0.00003}

# Tello velocity limits (cm/s)
MAX_VELOCITY = 35
