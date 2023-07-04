from pyrose_math.geometry import Twist3D
from kinematics import velocity


class MecanumWheelSpeeds:
    def __init__(self, frontLeft, frontRight, backLeft, backRight):
        if isinstance(frontLeft, velocity):
            self.frontLeft = frontLeft
        else:
            self.frontLeft = velocity(frontLeft)

        if isinstance(frontRight, velocity):
            self.frontRight = frontRight
        else:
            self.frontRight = velocity(frontRight)

        if isinstance(backLeft, velocity):
            self.backLeft = backLeft
        else:
            self.backLeft = velocity(backLeft)

        if isinstance(backRight, velocity):
            self.backRight = backRight
        else:
            self.backRight = velocity(backRight)


class DifferentialDriveWheelSpeeds:
    def __init__(self, left, right):
        if isinstance(left, velocity):
            self.left = left
        else:
            self.left = velocity(left)

        if isinstance(right, velocity):
            self.right = right
        else:
            self.right = velocity(right)


class MecanumKinematics:
    def __init__(self, track_width):
        self.track_width = track_width

    def wheelVelocityToTwistRobot(self, ws: MecanumWheelSpeeds) -> tuple:
        vx = (ws.frontLeft + ws.frontRight + ws.backLeft + ws.backRight) / 4
        vy = (ws.backLeft + ws.frontRight - ws.frontLeft - ws.backRight) / 4
        omega = (ws.backRight + ws.frontRight - ws.frontLeft - ws.backLeft) / (4 * self.track_width)
        return vx, vy, velocity(0), velocity(0), velocity(0), omega

    def robotTwistToWheelSpeeds(self, rv: Twist3D) -> MecanumWheelSpeeds:
        fl = rv.vx - rv.vy - (self.track_width * rv.wYaw)
        bl = rv.vx + rv.vy - (self.track_width * rv.wYaw)
        br = rv.vx - rv.vy + (self.track_width * rv.wYaw)
        fr = rv.vx + rv.vy + (self.track_width * rv.wYaw)
        return MecanumWheelSpeeds(fl, fr, bl, br)


class DifferentialDriveKinematics:
    def __init__(self, track_width):
        self.track_width = track_width

    def wheelVelocityToTwistRobot(self, ws: DifferentialDriveWheelSpeeds) -> Twist3D:
        vx = (ws.left + ws.right) / 2
        omega = (ws.right - ws.left) / self.track_width
        return Twist3D(vx, velocity(0), velocity(0), velocity(0), velocity(0), omega)

    def robotTwistToWheelSpeeds(self, rv: Twist3D) -> DifferentialDriveWheelSpeeds:
        vr = rv.vx + rv.wYaw * (self.track_width/2.0)
        vl = rv.vx - rv.wYaw * (self.track_width / 2.0)
        return DifferentialDriveWheelSpeeds(vl, vr)
