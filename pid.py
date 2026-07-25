from config import TRACKING_CONFIG


class PIDController:
    def __init__(self, kp, ki, kd, integral_max=50.0, lpf_alpha=0.2):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        self.prev_error = None
        self.integral = 0.0
        self.integral_max = abs(integral_max)
        self.lpf_alpha = lpf_alpha
        self.filtered_derivative = 0.0

    def calculate(self, error, dt=0.01):
        # dt 예외 처리
        if dt <= 0.0 or dt > 0.5:
            dt = 0.01

        # 미분 항 계산 (LPF 적용)
        if self.prev_error is None:
            raw_derivative = 0.0
            self.filtered_derivative = 0.0
        else:
            raw_derivative = (error - self.prev_error) / dt
            self.filtered_derivative = (
                self.lpf_alpha * raw_derivative
                + (1.0 - self.lpf_alpha) * self.filtered_derivative
            )

        # 적분 항 계산 및 Anti-windup
        if self.ki != 0:
            self.integral += error * dt
            i_term = self.ki * self.integral
            if abs(i_term) > self.integral_max:
                i_term = self.integral_max if i_term > 0 else -self.integral_max
                self.integral = i_term / self.ki
        else:
            i_term = 0.0

        # PID 제어량 산출
        output = (self.kp * error) + i_term + (self.kd * self.filtered_derivative)
        self.prev_error = error

        # 출력 제한 (Max/Min Speed 클리핑)
        max_speed = TRACKING_CONFIG.get("max_speed", 100.0)
        output = max(-max_speed, min(output, max_speed))

        return output

    def reset(self):
        """제어기 내부 상태 초기화 (타겟 손실 후 재추적 시 호출)"""
        self.prev_error = None
        self.integral = 0.0
        self.filtered_derivative = 0.0