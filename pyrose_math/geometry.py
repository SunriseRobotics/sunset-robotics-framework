import numpy as np
import math

from pyrose_math.kinematics import *


def matrix_exponential(mat):
    """
    Compute the matrix exponential using a power series.

    :param mat: numpy.ndarray, input matrix
    :return: numpy.ndarray, the exponential of the input matrix
    """
    # Identity matrix of the same shape as input matrix
    I = np.eye(*mat.shape)

    # Initialize result to the identity matrix
    result = np.copy(I)

    # Power of matrix
    mat_power = np.copy(mat)

    # Factorial term
    factorial = 1

    # Sum the series up to some finite number of terms
    # The larger this number, the more accurate the result will be
    for n in range(1, 20):
        factorial *= n
        result += mat_power / factorial
        mat_power = np.dot(mat_power, mat)

    return result

class Quaternion:
    def __init__(self, w, x, y, z):
        self.w = w
        self.x = x
        self.y = y
        self.z = z

    @classmethod
    def from_angle_axis(cls, theta, axis):
        """Create quaternion from angle-axis rotation."""
        axis = axis / np.linalg.norm(axis)  # Ensure axis is a unit vector
        w = np.cos(theta / 2)
        x, y, z = np.sin(theta / 2) * axis
        return cls(w, x, y, z)

    def __mul__(self, other):
        """Quaternion multiplication (i.e., composition of rotations)."""
        w1, x1, y1, z1 = self.w, self.x, self.y, self.z
        w2, x2, y2, z2 = other.w, other.x, other.y, other.z
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x2 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 + y2 * w2 + z1 * x2 - x1 * z2
        z = w1 * z2 + z2 * w2 + x1 * y2 - y1 * x2
        return Quaternion(w, x, y, z)

    def to_rotation_matrix(self):
        """Convert quaternion to rotation matrix."""
        w, x, y, z = self.w, self.x, self.y, self.z
        return np.array([
            [1 - 2 * y ** 2 - 2 * z ** 2, 2 * x * y - 2 * z * w, 2 * x * z + 2 * y * w],
            [2 * x * y + 2 * z * w, 1 - 2 * x ** 2 - 2 * z ** 2, 2 * y * z - 2 * x * w],
            [2 * x * z - 2 * y * w, 2 * y * z + 2 * x * w, 1 - 2 * x ** 2 - 2 * y ** 2]
        ])


class SO3:
    def __init__(self, rotation_matrix):
        self.rotation_matrix = rotation_matrix

    @classmethod
    def from_euler(cls, roll, pitch, yaw):
        R_x = np.array([[1, 0, 0],
                        [0, np.cos(roll), -np.sin(roll)],
                        [0, np.sin(roll), np.cos(roll)]])
        R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                        [0, 1, 0],
                        [-np.sin(pitch), 0, np.cos(pitch)]])
        R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                        [np.sin(yaw), np.cos(yaw), 0],
                        [0, 0, 1]])

        R = R_z @ R_y @ R_x
        return cls(R)

    def __mul__(self, other):
        if isinstance(other, SO3):
            return SO3(np.dot(self.rotation_matrix, other.rotation_matrix))  # order is important
        elif isinstance(other, np.ndarray):
            return np.dot(self.rotation_matrix, other)

    def inverse(self):
        return SO3(self.rotation_matrix.T)

    def rotate(self, vec):
        return np.dot(self.rotation_matrix, vec)

    def to_euler(self):
        """
        Convert the rotation matrix to Euler angles using the 'zyx' convention.
        """
        R = self.rotation_matrix
        yaw = np.arctan2(R[1, 0], R[0, 0])
        pitch = np.arctan2(-R[2, 0], np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2))
        roll = np.arctan2(R[2, 1], R[2, 2])

        return roll, pitch, yaw


class SE3:
    """
    Lie group to represent a 3D spatial transformation from a given reference frame
    """
    def __init__(self, rotation, translation):
        self.rotation = rotation
        self.translation = translation

    @classmethod
    def from_euler_and_translation(cls, roll, pitch, yaw, tx, ty, tz):
        rotation = SO3.from_euler(roll, pitch, yaw)
        translation = np.array([tx, ty, tz])
        return cls(rotation, translation)

    def __mul__(self, other):
        if isinstance(other, SE3):
            rotation = self.rotation * other.rotation
            translation = self.rotation * other.translation + self.translation
            return SE3(rotation, translation)
        elif isinstance(other, np.ndarray):
            return self.rotation * other + self.translation

    def inverse(self):
        """
        SE3 represents a transformation from some origin, this gives the SE3 representation to that origin
        """
        rotation_inv = self.rotation.inverse()
        translation_inv = -1 * rotation_inv.rotate(self.translation)
        return SE3(rotation_inv, translation_inv)

    def transform_to(self, other: 'SE3'):
        """
        transform from self to other expressed as SE3 lie group
        """
        return other * self.inverse()

    def relative_to(self, other: 'SE3'):
        other_to_origin = other.inverse()
        return other_to_origin * self

    @staticmethod
    def from_twist(twist):
        """
        Given a 6D twist vector, computes the change in position as a SE3 transformation.

        The twist vector should be in the following format:
        twist = [v_x, v_y, v_z, ω_x, ω_y, ω_z]
        where v is linear velocity and ω is angular velocity.
        """
        # Reshape the twist vector into a 4x4 se(3) matrix
        se3_mat = np.array([[0, -twist[5], twist[4], twist[0]],
                            [twist[5], 0, -twist[3], twist[1]],
                            [-twist[4], twist[3], 0, twist[2]],
                            [0, 0, 0, 0]])

        # Compute the matrix exponential to get the SE3 transformation
        SE3_mat = matrix_exponential(se3_mat)

        # Split into 3x3 rotation matrix and 3x1 translation vector
        rotation = SO3(SE3_mat[:3, :3])
        translation = SE3_mat[:3, 3]

        return SE3(rotation, translation)

    @staticmethod
    def from_angular_and_linear_velocities(linear_vel, roll_vel, pitch_vel, yaw_vel):
        """
        Given angular velocities (roll, pitch, yaw) and a 3D linear velocity vector,
        computes the change in position as a SE3 transformation.
        """
        # Compose the twist vector
        twist = np.array([linear_vel[0], linear_vel[1], linear_vel[2], roll_vel, pitch_vel, yaw_vel])

        # Call the from_twist method to get the SE3 transformation
        return SE3.from_twist(twist)


class Rotation3D:
    def __init__(self, roll: float, pitch: float, yaw: float) -> None:
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

    def __add__(self, other: 'Rotation3D'):
        return Rotation3D(self.roll + other.roll, self.pitch + other.pitch, self.yaw + other.yaw)

    def normalize_angles(self):
        self.roll = self.roll % (2 * math.pi)
        self.pitch = self.pitch % (2 * math.pi)
        self.yaw = self.yaw % (2 * math.pi)

    def __sub__(self, other: 'Rotation3D'):
        return Rotation3D(self.roll - other.roll, self.pitch - other.pitch, self.yaw - other.yaw)

    def __mul__(self, other: 'Rotation3D'):
        return Rotation3D(self.roll * other.roll, self.pitch * other.pitch, self.yaw * other.yaw)

    def __truediv__(self, other: 'Rotation3D'):
        return Rotation3D(self.roll / other.roll, self.pitch / other.pitch, self.yaw / other.yaw)

    def __floordiv__(self, other: 'Rotation3D'):
        return Rotation3D(self.roll // other.roll, self.pitch // other.pitch, self.yaw // other.yaw)

    def __mod__(self, other: 'Rotation3D'):
        return Rotation3D(self.roll % other.roll, self.pitch % other.pitch, self.yaw % other.yaw)

    def __pow__(self, other: 'Rotation3D'):
        return Rotation3D(self.roll ** other.roll, self.pitch ** other.pitch, self.yaw ** other.yaw)

    def __eq__(self, other: 'Rotation3D'):
        self.normalize_angles()
        other.normalize_angles()
        return self.roll == other.roll and self.pitch == other.pitch and self.yaw == other.yaw


class Pose3D:
    def __init__(self, x: position, y: position, z: position, rotation: Rotation3D) -> None:
        self.x = x
        self.y = y
        self.z = z
        self.rotation = rotation

    def __add__(self, other: 'Pose3D'):
        return Pose3D(self.x + other.x, self.y + other.y, self.z + other.z, self.rotation + other.rotation)

    def __sub__(self, other: 'Pose3D'):
        return Pose3D(self.x - other.x, self.y - other.y, self.z - other.z, self.rotation - other.rotation)

    def __mul__(self, other: 'Pose3D'):
        return Pose3D(self.x * other.x, self.y * other.y, self.z * other.z, self.rotation * other.rotation)

    def __truediv__(self, other: 'Pose3D'):
        return Pose3D(self.x / other.x, self.y / other.y, self.z / other.z, self.rotation / other.rotation)

    def __floordiv__(self, other: 'Pose3D'):
        return Pose3D(self.x // other.x, self.y // other.y, self.z // other.z, self.rotation // other.rotation)


class Twist3D:
    def __init__(self, vx: velocity, vy: velocity, vz: velocity, wRoll: velocity, wPitch: velocity, wYaw: velocity):
        self.vx = vx
        self.vy = vy
        self.vz = vz
        self.wRoll = wRoll
        self.wPitch = wPitch
        self.wYaw = wYaw

    def __add__(self, other: 'Twist3D'):
        return Twist3D(self.vx + other.vx, self.vy + other.vy, self.vz + other.vz, self.wRoll + other.wRoll,
                       self.wPitch + other.wPitch, self.wYaw + other.wYaw)

    def __sub__(self, other: 'Twist3D'):
        return Twist3D(self.vx - other.vx, self.vy - other.vy, self.vz - other.vz, self.wRoll - other.wRoll,
                       self.wPitch - other.wPitch, self.wYaw - other.wYaw)

    def twist_to_pose(self, time: tSeconds) -> 'Pose3D':
        # Integrate linear velocities
        dx = self.vx.getAsPositionDelta(time)
        dy = self.vy.getAsPositionDelta(time)
        dz = self.vz.getAsPositionDelta(time)
        translation = vec3(dx, dy, dz)

        # Convert angular velocities to quaternion
        angular_velocity = velocity(np.sqrt(self.wRoll.get() ** 2 + self.wPitch.get() ** 2 + self.wYaw.get() ** 2))
        axis_of_rotation = np.array([self.wRoll, self.wPitch, self.wYaw]) / angular_velocity
        dtheta = angular_velocity * time.seconds
        rotation = Quaternion.from_angle_axis(dtheta, axis_of_rotation)

        return Pose3D(translation.x,translation.y,translation.z, rotation)


class Twist2D:
    def __init__(self, vx: velocity, vy: velocity, wTheta: velocity):
        self.vx = vx
        self.vy = vy
        self.wTheta = wTheta

    def __add__(self, other: 'Twist2D'):
        return Twist2D(self.vx + other.vx, self.vy + other.vy, self.wTheta + other.wTheta)

    def __sub__(self, other: 'Twist2D'):
        return Twist2D(self.vx - other.vx, self.vy - other.vy, self.wTheta - other.wTheta)

    def twist_to_pose(self, time: tSeconds) -> 'Pose2D':
        # Integrate angular velocity to get change in orientation
        theta = self.wTheta * time.seconds

        # If angular velocity is zero, we are moving in a straight line
        if abs(self.wTheta) < 1e-9:
            dx = self.vx.getAsPositionDelta(time)
            dy = self.vy.getAsPositionDelta(time)
        else:
            # Otherwise, we are moving along a circular arc
            radius = np.sqrt(self.vx.get() ** 2 + self.vy.get() ** 2) / self.wTheta  # radius of curvature
            dx = radius * np.sin(theta)  # change in x position
            dy = radius * (1 - np.cos(theta))  # change in y position
            dx = position(dx)
            dy = position(dy)
        return Pose2D(dx, dy, theta)

    def twist_to_pose_dtheta(self, time: tSeconds, dtheta):
        # Integrate angular velocity to get change in orientation
        theta = dtheta

        # If angular velocity is zero, we are moving in a straight line
        if abs(self.wTheta) < 1e-9:
            dx = self.vx.getAsPositionDelta(time)
            dy = self.vy.getAsPositionDelta(time)
        else:
            # Otherwise, we are moving along a circular arc
            radius = np.sqrt(self.vx.get() ** 2 + self.vy.get() ** 2) / self.wTheta  # radius of curvature
            dx = radius * np.sin(theta)  # change in x position
            dy = radius * (1 - np.cos(theta))  # change in y position
            dx = position(dx)
            dy = position(dy)
        return Pose2D(dx, dy, theta)


class Pose2D:
    def __init__(self, x: position, y: position, theta: float):
        self.x = x
        self.y = y
        self.theta = theta

    def rotateThenAdd(self, other: 'Pose2D'):
        # Rotate the translation part of 'other' by the rotation part of 'self', then add to self's translation
        newX = self.x.get() + other.x.get() * math.cos(self.theta) - other.y.get() * math.sin(self.theta)
        newY = self.y.get() + other.x.get() * math.sin(self.theta) + other.y.get() * math.cos(self.theta)

        # Add the rotations
        newTheta = self.theta + other.theta

        return Pose2D(position(newX), position(newY), newTheta)

    def __add__(self, other: 'Pose2D'):
        return Pose2D(self.x + other.x, self.y + other.y, self.theta + other.theta)

    def __sub__(self, other: 'Pose2D'):
        return Pose2D(self.x - other.x, self.y - other.y, self.theta - other.theta)

