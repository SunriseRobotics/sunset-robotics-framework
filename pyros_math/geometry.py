import numpy as np
import math 

from pyros_math.kinematics import *

class twist2D:
    def __init__(self,vx: velocity, vy: velocity, wz: velocity):
        self.vx = vx
        self.vy = vy
        self.wz = wz

    def __add__(self, other: 'twist2D'):
        return twist2D(self.vx + other.vx, self.vy + other.vy, self.wz + other.wz)

    def __sub__(self, other: 'twist2D'):
        return twist2D(self.vx - other.vx, self.vy - other.vy, self.wz - other.wz)
    def twist_to_transform(self, time: tSeconds) -> 'transform2D':
        # Integrate angular velocity to get change in orientation
        theta = self.wz * time.seconds

        # If angular velocity is zero, we are moving in a straight line
        if abs(self.wz) < 1e-9:
            dx = self.vx.getAsPositionDelta(time)
            dy = self.vy.getAsPositionDelta(time)
        else:
            # Otherwise, we are moving along a circular arc
            radius = np.sqrt(self.vx.get()**2 + self.vy.get()**2) / self.wz  # radius of curvature
            dx = radius * np.sin(theta)  # change in x position
            dy = radius * (1 - np.cos(theta))  # change in y position
            dx = position(dx)
            dy = position(dy)
        return transform2D(dx, dy, theta)
    
    def twist_to_transform_dtheta(self, time: tSeconds, dtheta):
          # Integrate angular velocity to get change in orientation
        theta = dtheta

        # If angular velocity is zero, we are moving in a straight line
        if abs(self.wz) < 1e-9:
            dx = self.vx.getAsPositionDelta(time)
            dy = self.vy.getAsPositionDelta(time)
        else:
            # Otherwise, we are moving along a circular arc
            radius = np.sqrt(self.vx.get()**2 + self.vy.get()**2) / self.wz  # radius of curvature
            dx = radius * np.sin(theta)  # change in x position
            dy = radius * (1 - np.cos(theta))  # change in y position
            dx = position(dx)
            dy = position(dy)
        return transform2D(dx, dy, theta)      
    
    
class transform2D:
    def __init__(self, x: position, y: position, theta: float):
        self.x = x
        self.y = y
        self.theta = theta

    def rotateThenAdd(self, other: 'transform2D'):
        # Rotate the translation part of 'other' by the rotation part of 'self', then add to self's translation
        newX = self.x.get() + other.x.get() * math.cos(self.theta) - other.y.get() * math.sin(self.theta)
        newY = self.y.get() + other.x.get() * math.sin(self.theta) + other.y.get() * math.cos(self.theta)

        # Add the rotations
        newTheta = self.theta + other.theta

        return transform2D(position(newX), position(newY), newTheta)

    def __add__(self, other: 'transform2D'):
        return transform2D(self.x + other.x, self.y + other.y, self.theta + other.theta)

    def __sub__(self, other: 'transform2D'):
        return transform2D(self.x - other.x, self.y - other.y, self.theta - other.theta)



