from pyrose_math.geometry import SE3, SO3


class MecanumWheelSpeeds:
    """
    Wheel speeds for a mecanum drive holonomic mobile robot
    """
    def __init__(self, frontLeft, frontRight, backLeft, backRight):
        self.frontLeft = frontLeft
        self.frontRight = frontRight
        self.backLeft = backLeft
        self.backRight = backRight


class DifferentialDriveWheelSpeeds:
    """
    Wheel speeds for a differential steering mobile robot.

    If using multiple wheels on each side, average each wheel on that side to estimate that sides corresponding value.
    """
    def __init__(self, left, right):
        self.left = left
        self.right = right


class MecanumKinematics:
    """
    Convert from between twists and wheel velocities for a mecanum drive robot.
    """
    def __init__(self, track_width):
        """
        track width must be in the same units as your wheel velocity.
        """
        self.track_width = track_width

    def wheelVelocityToTwistRobot(self, ws: MecanumWheelSpeeds) -> tuple:
        vx = (ws.frontLeft + ws.frontRight + ws.backLeft + ws.backRight) / 4
        vy = (ws.backLeft + ws.frontRight - ws.frontLeft - ws.backRight) / 4
        omega = (ws.backRight + ws.frontRight - ws.frontLeft - ws.backLeft) / (4 * self.track_width)
        return vx, vy, 0, 0, 0, omega

    def robotTwistToWheelSpeeds(self, rv: SE3) -> MecanumWheelSpeeds:
        vx, vy, wYaw = rv.translation[0], rv.translation[1], rv.rotation.to_euler()[2]
        fl = vx - vy - (self.track_width * wYaw)
        bl = vx + vy - (self.track_width * wYaw)
        br = vx - vy + (self.track_width * wYaw)
        fr = vx + vy + (self.track_width * wYaw)
        return MecanumWheelSpeeds(fl, fr, bl, br)


class DifferentialDriveKinematics:
    """
    Convert from between twists and wheel velocities for a differential drive robot.
    """
    def __init__(self, track_width):
        """
        track width must be in the same units as your wheel velocity.
        """
        self.track_width = track_width

    def wheelVelocityToTwistRobot(self, ws: DifferentialDriveWheelSpeeds) -> tuple:
        vx = (ws.left + ws.right) / 2
        omega = (ws.right - ws.left) / self.track_width
        return vx, 0, 0, 0, 0, omega

    def robotTwistToWheelSpeeds(self, rv: SE3) -> DifferentialDriveWheelSpeeds:
        vx = rv.translation[0]
        wYaw = rv.rotation.to_euler()[2]
        vr = vx + wYaw * (self.track_width / 2.0)
        vl = vx - wYaw * (self.track_width / 2.0)
        return DifferentialDriveWheelSpeeds(vl, vr)
