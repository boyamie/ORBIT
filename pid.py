from config import TRACKING_CONFIG


class PIDController:
    def __init__(self, kp, ki, kd):
    
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.prev_error = 0.0
        self.integral = 0.0

    def calculate(self, error):
        self.integral += error

        derivative = error - self.prev_error

        output = (
            self.kp * error
            + self.ki * self.integral
            + self.kd * derivative
        )

        self.prev_error = error

        max_speed = TRACKING_CONFIG["max_speed"]
        min_speed = TRACKING_CONFIG["min_speed"]

        output = max(min(output, max_speed), min_speed)

        return output########