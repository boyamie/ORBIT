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

    def calculate(self, error, dt=0.033):
        # 1. dt 예외 처리 보완 (Tello 카메라 프레임 속도에 현실적으로 맞춤)
        # dt가 0 이하이거나, 0.2초(5 FPS 이하로 심하게 딜레이된 경우)를 넘어가면 0.033초(30 FPS)로 보정
        if dt <= 0.0 or dt > 0.2:
            dt = 0.033

        # 2. 미분 항 계산 (LPF 적용)
        if self.prev_error is None:
            raw_derivative = 0.0
            self.filtered_derivative = 0.0
        else:
            raw_derivative = (error - self.prev_error) / dt
            self.filtered_derivative = (
                self.lpf_alpha * raw_derivative
                + (1.0 - self.lpf_alpha) * self.filtered_derivative
            )

        # 3. 적분 항 계산 및 Anti-windup
        if self.ki != 0:
            self.integral += error * dt
            i_term = self.ki * self.integral
            if abs(i_term) > self.integral_max:
                i_term = self.integral_max if i_term > 0 else -self.integral_max
                self.integral = i_term / self.ki
        else:
            i_term = 0.0

        # 4. PID 제어량 산출
        output = (self.kp * error) + i_term + (self.kd * self.filtered_derivative)
        self.prev_error = error

        # 5. 출력 제한 (Tello 최대 속도 제한)
        max_speed = TRACKING_CONFIG.get("max_speed", 100.0)
        output = max(-max_speed, min(output, max_speed))

        # 6. Tello RC 전송용 정수(Int) 형변환
        return int(round(output))

    def reset(self):
        """제어기 내부 상태 초기화 (타겟 손실 후 재추적 시 호출)"""
        self.prev_error = None
        self.integral = 0.0
        self.filtered_derivative = 0.0