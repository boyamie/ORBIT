from config import TRACKING_CONFIG


class PIDController:
    def __init__(self, kp, ki, kd, integral_max=50.0, lpf_alpha=0.2, deadband=5.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.prev_error = None
        self.integral = 0.0
        self.integral_max = abs(integral_max)
        self.lpf_alpha = lpf_alpha
        self.filtered_derivative = 0.0
        self.deadband = deadband  # 1. 사감대(Deadband) 범위 설정 (픽셀 단위)

    def calculate(self, error, dt=0.033):
        # [수정 1] Deadband 처리 - 오차가 미세하면 0으로 취급하여 덜덜거림 방지
        if abs(error) < self.deadband:
            error = 0.0

        # [수정 2] dt 예외 처리 및 프레임 스킵 대응 (미분 튀는 현상 방지)
        dt_corrupted = False
        if dt <= 0.0 or dt > 0.2:
            dt = 0.033
            dt_corrupted = True

        # [수정 3] 미분 항 계산 (LPF 적용 및 안전장치)
        if self.prev_error is None or dt_corrupted:
            raw_derivative = 0.0
            self.filtered_derivative = 0.0
        else:
            raw_derivative = (error - self.prev_error) / dt
            self.filtered_derivative = (
                self.lpf_alpha * raw_derivative
                + (1.0 - self.lpf_alpha) * self.filtered_derivative
            )

        # [수정 4] 적분 항 계산 및 Anti-windup
        if self.ki != 0:
            self.integral += error * dt
            i_term = self.ki * self.integral

            # Integral Clamping
            if abs(i_term) > self.integral_max:
                i_term = self.integral_max if i_term > 0 else -self.integral_max
                self.integral = i_term / self.ki
        else:
            i_term = 0.0

        # PID 비례/적분/미분 계산
        p_term = self.kp * error
        d_term = self.kd * self.filtered_derivative
        output = p_term + i_term + d_term

        self.prev_error = error

        # [수정 5] 속도 출력 제한 (Tello 최대 속도 제한)
        max_speed = TRACKING_CONFIG.get("max_speed", 100.0)
        output = max(-max_speed, min(output, max_speed))

        # Tello RC 명령용 정수 반환
        return int(round(output))

    def reset(self):
        """제어기 내부 상태 초기화 (타겟 손실 후 재추적 시 반드시 호출)"""
        self.prev_error = None
        self.integral = 0.0
        self.filtered_derivative = 0.0