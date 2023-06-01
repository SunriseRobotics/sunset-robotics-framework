import numpy as np
import unittest
import math 

from kinematics import *

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


class TestTwist2DMethods(unittest.TestCase):
    def test_twist_to_transform_straight_line(self):
        twist = twist2D(velocity(5), velocity(0), 0)
        time = seconds(1)
        transform = twist.twist_to_transform(time)
        self.assertAlmostEqual(transform.x.get(), 5)
        self.assertAlmostEqual(transform.y.get(), 0)
        self.assertAlmostEqual(transform.theta, 0)

    def test_twist_to_transform_circular_path(self):
        twist = twist2D(vx=velocity(0), vy=velocity(5), wz=math.pi/2)
        transform = twist.twist_to_transform(seconds(1))

        # The expected position would be (10/pi, 10/pi)
        self.assertAlmostEqual(transform.x.get(), 3.183098861837907)
        self.assertAlmostEqual(transform.y.get(), 3.183098861837907)

    def test_twist_to_transform_zero_time(self):
        twist = twist2D(velocity(5), velocity(0), math.pi/2)
        time = seconds(0)
        transform = twist.twist_to_transform(time)
        self.assertAlmostEqual(transform.x.get(), 0)
        self.assertAlmostEqual(transform.y.get(), 0)
        self.assertAlmostEqual(transform.theta, 0)


if __name__ == '__main__':
    unittest.main()
