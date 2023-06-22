import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class TriadVector:
    def __init__(self, ax, origin=np.array([0, 0, 0]), length=1.0):
        self.ax = ax
        self.origin = origin
        self.length = length
        self.unit_vectors = {'x': np.array([1, 0, 0]), 'y': np.array([0, 1, 0]), 'z': np.array([0, 0, 1])}
        self.colors = {'x': 'r', 'y': 'g', 'z': 'b'}

        # Create a dictionary of arrows
        self.arrows = {hat: self.ax.quiver(*self.origin, *self.length * uv, color=color) for hat, uv, color in 
                       zip(self.unit_vectors.keys(), self.unit_vectors.values(), self.colors.values())}

        self.bone = None
        self.other_triad = None 

    def add_bone(self, other_triad):

        if self.bone is not None:
            self.bone.remove() 

        self.other_triad = other_triad
        self.bone = self.ax.plot(
            [self.origin[0], other_triad.origin[0]],
            [self.origin[1], other_triad.origin[1]],
            [self.origin[2], other_triad.origin[2]],
            color='k',
            zdir='z'
        )[0]
        other_triad.bone = self.bone

    def set_position(self, x, y, z):
        # use translate since we know that works well

        # calculate offset
        dx = x - self.origin[0]
        dy = y - self.origin[1]
        dz = z - self.origin[2]

        # translate
        return self.translate(dx, dy, dz)
        

    def translate(self, dx, dy, dz):
        self.origin += np.array([dx, dy, dz])

        for hat in self.arrows.keys():
            self.arrows[hat].remove()
            self.arrows[hat] = self.ax.quiver(*self.origin, *self.length * self.unit_vectors[hat], color=self.colors[hat])

        if self.bone is not None:
            xdata, ydata, zdata = self.bone.get_data_3d()
            self.bone.set_data(np.array([self.origin[0], xdata[1]]), np.array([self.origin[1], ydata[1]]))
            self.bone.set_3d_properties(np.array([self.origin[2], zdata[1]]))

        artists = list(self.arrows.values())


        if self.bone is not None and self.other_triad is not None:
            self.add_bone(self.other_triad)
            artists.append(self.bone)



        return artists
    
if __name__ == "__main__":
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim([-2, 2])
    ax.set_ylim([-2, 2])
    ax.set_zlim([-2, 2])
    rotation_matrix1 = np.eye(3)
    rotation_matrix2 = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])

    triad1 = TriadVector(ax, [0, 0, 0], rotation_matrix1)
    triad2 = TriadVector(ax, [1, 1, 1], rotation_matrix2)
    triad2.add_bone(triad1)
    triads = [triad1, triad2]

    def update(_):
        artists = []
        for triad in triads:
            triad_artists = triad.set_position(np.random.randn(), np.random.randn(), np.random.randn())
            artists.extend(triad_artists)
        return artists

    ani = FuncAnimation(fig, update, frames=10000, blit=False, interval=10, repeat=True)
    plt.show()
