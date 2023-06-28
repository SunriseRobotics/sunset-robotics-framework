from matplotlib.animation import FuncAnimation

import numpy as np
import matplotlib.pyplot as plt


class TriadVector:
    def __init__(self, ax, origin, length=1.0):
        self.ax = ax
        self.origin = np.array(origin)
        self.length = length
        self.unit_vectors = {'x': np.array([1, 0, 0]), 'y': np.array([0, 1, 0]), 'z': np.array([0, 0, 1])}
        self.colors = {'x': 'r', 'y': 'g', 'z': 'b'}
        self.rotation_data = None
        # Create a dictionary of lines
        self.lines = {hat: self.ax.plot3D(*zip(self.origin, self.origin + self.length * uv), color=color)[0] for
                      hat, uv, color in zip(self.unit_vectors.keys(), self.unit_vectors.values(), self.colors.values())}

    def set_position(self, position):
        self.origin = np.array(position)
        # Update lines' positions
        for hat, uv in self.unit_vectors.items():
            # Use current rotation_data to calculate the orientation
            if self.rotation_data is not None:
                rotation_matrix = self.rotation_matrix(*self.rotation_data)
                rotated_uv = rotation_matrix @ uv
            else:
                rotated_uv = uv
            # Update lines
            self.update_lines(hat, rotated_uv)

        # implement

    def set_rotation(self, angles):
        # Generate rotation matrix from Euler angles
        rotation_matrix = self.rotation_matrix(*angles)
        # Rotate unit vectors
        for hat, uv in self.unit_vectors.items():
            rotated_uv = rotation_matrix @ uv
            # Update lines
            self.update_lines(hat, rotated_uv)

    def update_lines(self, hat, rotated_uv):
        self.lines[hat].set_xdata(np.array([self.origin[0], self.origin[0] + self.length * rotated_uv[0]]))
        self.lines[hat].set_ydata(np.array([self.origin[1], self.origin[1] + self.length * rotated_uv[1]]))
        self.lines[hat].set_3d_properties(np.array([self.origin[2], self.origin[2] + self.length * rotated_uv[2]]))

    def get_artists(self):
        return list(self.lines.values())

    @staticmethod
    def rotation_matrix(x, y, z):
        cos_x = np.cos(x)
        sin_x = np.sin(x)
        cos_y = np.cos(y)
        sin_y = np.sin(y)
        cos_z = np.cos(z)
        sin_z = np.sin(z)

        rotation_x = np.array([[1, 0, 0], [0, cos_x, -sin_x], [0, sin_x, cos_x]])
        rotation_y = np.array([[cos_y, 0, sin_y], [0, 1, 0], [-sin_y, 0, cos_y]])
        rotation_z = np.array([[cos_z, -sin_z, 0], [sin_z, cos_z, 0], [0, 0, 1]])

        # Combine rotation matrices in the order of z, y, and x
        rotation_matrix = rotation_z @ rotation_y @ rotation_x

        return rotation_matrix


if __name__ == "__main__":

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Create triad vectors
    triads = [TriadVector(ax, origin=[i - 1, i - 1, i - 1]) for i in range(3)]


    def update(frame):
        for i, triad in enumerate(triads):
            # Give each vector a slightly different rotation speed
            triad.set_rotation([(i + 1) * frame / 50, (i + 1) * frame / 100, (i + 1) * frame / 150])
        ax.autoscale()


    ani = FuncAnimation(fig, update, frames=np.linspace(0, 2 * np.pi, 200), blit=False, interval=100, repeat=True)
    plt.show()
