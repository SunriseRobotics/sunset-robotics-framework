from pyrose_math.geometry import SE3, SO3


class MecanumWheelSpeeds:
    def __init__(self, frontLeft, frontRight, backLeft, backRight):
        self.frontLeft = frontLeft
        self.frontRight = frontRight
        self.backLeft = backLeft
        self.backRight = backRight


class DifferentialDriveWheelSpeeds:
    def __init__(self, left, right):
        self.left = left
        self.right = right


class MecanumKinematics:
    def __init__(self, track_width):
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
    def __init__(self, track_width):
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
