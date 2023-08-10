import numpy as np


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

        # Normalize the yaw to be in the range [-pi, pi]
        if yaw > np.pi:
            yaw -= 2 * np.pi
        elif yaw < -np.pi:
            yaw += 2 * np.pi

        return roll, pitch, yaw

    def to_message_dict(self):
        roll, pitch, yaw = self.to_euler()
        return {"ROLL": roll, "PITCH": pitch, "YAW": yaw}

    def __add__(self, other):
        if not isinstance(other, SO3):
            raise TypeError("Other must be an SO3 instance")


class SE3:
    """
    Lie group to represent a 3D spatial transformation from a given reference frame
    """

    def __init__(self, rotation: SO3, translation):
        self.rotation = rotation
        self.translation = translation

    @classmethod
    def from_euler_and_translation(cls, roll, pitch, yaw, tx, ty, tz):
        rotation = SO3.from_euler(roll, pitch, yaw)
        translation = np.array([tx, ty, tz])
        return cls(rotation, translation)

    @classmethod
    def from_message_dict(cls, message):
        roll, pitch, yaw = message["ROLL"], message["PITCH"], message["YAW"]
        tx, ty, tz = message["X"], message["Y"], message["Z"]
        rotation = SO3.from_euler(roll, pitch, yaw)
        translation = np.array([tx, ty, tz])
        return cls(rotation,translation)

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

    def to_message_dict(self):
        roll, pitch, yaw = self.rotation.to_euler()
        x, y, z = self.translation[0], self.translation[1], self.translation[2]
        return {"X": x, "Y": y, "Z": z, "ROLL": roll, "PITCH": pitch, "YAW": yaw}


    def rotate_around(self, other: "SE3"):
        """
        Using the other SE3's position as the origin, rotate this by the others orientation as well.
        """
        # Shift the world so that the other object is at the origin
        shifted = self.relative_to(other)

        # Apply the rotation from the other object
        rotated_position = other.rotation.rotate(shifted.translation)
        rotated = SE3(shifted.rotation * other.rotation, rotated_position)

        # Shift the world back
        result = other * rotated

        return result


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
