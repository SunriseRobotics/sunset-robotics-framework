import numpy as np


class PID:
    def __init__(self, kp, ki, kd):
        self.kP = kp
        self.kI = ki
        self.kD = kd
        self.previous_error = 0
        self.first_run = True

    def update(self, reference: float, estimated_state: float, timedelta: float):
        error = reference - estimated_state
        derivative = (error - self.previous_error) / timedelta
        if self.first_run:
            derivative = 0
        integral = (error + self.previous_error) / 2
        integral *= timedelta

        return self.kP * error + self.kI * integral + self.kD * derivative


class SimpleFeedforward:
    def __init__(self, kv, ka, static=0):
        self.kV = kv
        self.kA = ka
        self.kStatic = static

    def update(self, referenceVelocity, referenceAcceleration):
        return referenceVelocity * self.kV + referenceAcceleration * self.kA + np.sign(referenceVelocity) * self.kStatic


def calculateDerivativePositionControl(kp, kv, ka):
    """
    Given kv,ka, and your choice of kp, compute the optimal derivative gain
    """
    return max(2 * np.sqrt(ka * kp) - kv, 0)


