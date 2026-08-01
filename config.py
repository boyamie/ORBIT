TRACKING_CONFIG = {

    "yaw": {
        "kp": 0.45,
        "ki": 0.005,
        "kd": 0.30,
    },

    "forward": {
        "kp": 0.45,
        "ki": 0.003,
        "kd": 0.35,
    },

    "updown": {
        "kp": 0.50,
        "ki": 0.005,
        "kd": 0.30,
    },


    # 사람이 화면에서 차지하는 목표 면적
    "target_area": 150000,

    # target_area ± tolerance 안에서는 전후 이동하지 않음
    "area_tolerance": 12000,

    # 화면 중심 오차 허용범위(px)
    "center_deadzone": 25,

    # PID 출력이 한 프레임에 너무 크게 변하지 않도록 제한
    "max_delta_speed": 8,

    # RC 최대 속도
    "max_speed": 35,
    "min_speed": -35,
}