import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D

class TriadVector:
    def __init__(self, ax, origin, length=1.0):
        self.ax = ax
        self.origin = np.array(origin)
        self.length = length
        self.unit_vectors = {'x': np.array([1, 0, 0]), 'y': np.array([0, 1, 0]), 'z': np.array([0, 0, 1])}
        self.colors = {'x': 'r', 'y': 'g', 'z': 'b'}

        # Create a dictionary of arrows
        self.arrows = {hat: self.ax.quiver(*self.origin, *self.length * uv, color=color) for hat, uv, color in zip(self.unit_vectors.keys(), self.unit_vectors.values(), self.colors.values())}

    def rotate(self, angles):
        # Generate rotation matrix from Euler angles
        rotation_matrix = self.rotation_matrix(*angles)

        # Rotate unit vectors
        self.unit_vectors = {hat: rotation_matrix @ uv for hat, uv in self.unit_vectors.items()}

        # Update arrows
        for hat, uv in self.unit_vectors.items():
            self.arrows[hat].remove()
            self.arrows[hat] = self.ax.quiver(*self.origin, *self.length * uv, color=self.colors[hat])
    
    def set_rotation(self, angles):
        # Generate rotation matrix from Euler angles
        rotation_matrix = self.rotation_matrix(*angles)

        # Initialize unit vectors
        initial_unit_vectors = {'x': np.array([1, 0, 0]), 'y': np.array([0, 1, 0]), 'z': np.array([0, 0, 1])}

        # Rotate initial unit vectors
        self.unit_vectors = {hat: rotation_matrix @ uv for hat, uv in initial_unit_vectors.items()}

        # Update arrows
        for hat, uv in self.unit_vectors.items():
            self.arrows[hat].remove()
            self.arrows[hat] = self.ax.quiver(*self.origin, *self.length * uv, color=self.colors[hat])

    
    def get_artists(self):
        return list(self.arrows.values())
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
    triads = [TriadVector(ax, origin=[i-1, i-1, i-1]) for i in range(3)]

    def update(frame):
        for i, triad in enumerate(triads):
            # Give each vector a slightly different rotation speed
            triad.rotate([(i+1)*frame/50, (i+1)*frame/100, (i+1)*frame/150])
        ax.autoscale()

    ani = FuncAnimation(fig, update, frames=np.linspace(0, 2*np.pi, 200), blit=False, interval=100, repeat=True)
    plt.show()
