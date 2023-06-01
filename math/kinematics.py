from abc import ABC,abstractmethod
import numpy as np
import unittest
import math 

class state1D(ABC):
    def __init__(self) -> None:
        super().__init__()

    @abstractmethod
    def updateViaDelta(self, delta: 'state1D'):
        pass


    @abstractmethod
    def __mul__(self, other):
        pass

    @abstractmethod
    def __sub__(self, other):
        pass

    @abstractmethod
    def get(self):
        pass


class tSeconds:
    def __init__(self, seconds : float):
        self.seconds = seconds 

    def __add__(self, others: 'tSeconds'):
        return tSeconds(self.seconds + others.seconds)
    
    def __sub__(self, others: 'tSeconds'):
        return tSeconds(self.seconds - others.seconds)


def seconds(seconds: float) -> tSeconds:
    return tSeconds(seconds)

def milliseconds(milliseconds: float) -> tSeconds:
    return tSeconds(milliseconds / 1000)

def microseconds(microseconds: float) -> tSeconds:
    return tSeconds(microseconds / 1000000)

class velocity(state1D):
    def __init__(self,v) -> None:
        self.v = v
    def updateViaDelta(self, dv: 'velocity'):
        self.v += dv 
    def __mul__(self, other: float):
        return velocity(self.v * other)
    def __add__(self, other: 'velocity'):
        return velocity(self.v + other.get())
    def __sub__(self, other):
        if isinstance(other, velocity):
            return velocity(self.v - other.get())
        elif isinstance(other, (int, float)):
            return velocity(self.v - other)
        else:
            raise TypeError("unsupported operand type(s) for -: 'velocity' and '{}'".format(type(other)))
    def get(self):
        return self.v 
    def getAsPositionDelta(self, time: tSeconds) -> 'position':
        return position(self.v * time.seconds)
    

class position(state1D):
    def __init__(self, x) -> None:
        self.x = x
    def updateViaDelta(self,dx: 'position'):
        self.x += dx.get()
    def updateV(self,v: velocity, dt):
        self.x += v * dt
    def __mul__(self, other: float):
        return velocity(self.x * other)
    def __add__(self, other: 'position'):
        return position(self.x + other.get())
    
    def __sub__(self, other: 'position'):
        return position(self.x - other.get())

    def get(self):
        return self.x 



class vec2:
    def __init__(self,x: state1D, y: state1D):
        self.x = x
        self.y = y
        if not issubclass(type(x), state1D) or not issubclass(type(y), state1D): 
            raise TypeError(f"x (type: {type(x)}) and y (type: {type(y)}) must be subclasses of 'state' ") 
        if type(x) != type(y):
            raise TypeError(f"the state type of x ({type(x)}) must match the state type of y ({type(y)})")
        self.vec_np = np.array([x.get(),y.get()])
    def norm(self):
        return np.linalg.norm(self.vec_np)
    
    def unit(self):
        return self.vec_np * (1/self.norm())
    
    def typeMatches(self, other: 'vec2'):
        return type(self.x) == type(other.y)

    def __throw_if_mismatch__(self, other: 'vec2'):
        if not self.typeMatches(other):
            raise TypeError("Operation can not occur, likely due to missmatched time derivatives")
    
    def __reset_np_vec__(self):
        self.vec_np[0] = self.x.get()
        self.vec_np[1] = self.y.get()

    def add(self, other: 'vec2'):
        self.__throw_if_mismatch__(other)
        self.x.updateViaDelta(other.x)
        self.y.updateViaDelta(other.y)
        self.__reset_np_vec__()

    def sub(self,other: 'vec2'):
        self.__throw_if_mismatch__(other)
        self.x.updateViaDelta(other.x * -1)
        self.y.updateViaDelta(other.y * -1)
        self.__reset_np_vec__()
        

    def __add__(self, other: 'vec2'):
        return vec2(self.x + other.x, self.y + other.y)
    

    def __sub__(self, other: 'vec2'):
        return vec2(self.x - other.x, self.y - other.y) 







class TestStateMethods(unittest.TestCase):

    def test_seconds(self):
        self.assertEqual(seconds(5).seconds, 5)

    def test_milliseconds(self):
        self.assertAlmostEqual(milliseconds(5000).seconds, 5)

    def test_microseconds(self):
        self.assertAlmostEqual(microseconds(5000000).seconds, 5)

    def test_tSeconds_add(self):
        time1 = seconds(5)
        time2 = seconds(7)
        self.assertEqual((time1 + time2).seconds, 12)

    def test_tSeconds_sub(self):
        time1 = seconds(5)
        time2 = seconds(7)
        self.assertEqual((time2 - time1).seconds, 2)

    def test_velocity_mul(self):
        v = velocity(3)
        self.assertEqual((v * 2).get(), 6)

    def test_velocity_add(self):
        v1 = velocity(3)
        v2 = velocity(5)
        self.assertEqual((v1 + v2).get(), 8)

    def test_velocity_sub(self):
        v1 = velocity(3)
        v2 = velocity(5)
        self.assertEqual((v2 - v1).get(), 2)

    def test_position_mul(self):
        p = position(3)
        self.assertEqual((p * 2).get(), 6)

    def test_position_add(self):
        p1 = position(3)
        p2 = position(5)
        self.assertEqual((p1 + p2).get(), 8)

    def test_position_sub(self):
        p1 = position(3)
        p2 = position(5)
        self.assertEqual((p2 - p1).get(), 2)

    def test_vec2_add(self):
        v1 = vec2(position(1), position(2))
        v2 = vec2(position(2), position(3))
        v1.add(v2)
        self.assertEqual(v1.x.get(), 3)
        self.assertEqual(v1.y.get(), 5)
    
    def test_vec2_sub(self):
        v1 = vec2(position(1), position(2))
        v2 = vec2(position(2), position(3))
        v1.sub(v2)
        self.assertEqual(v1.x.get(), -1)
        self.assertEqual(v1.y.get(), -1)
    


if __name__ == '__main__':
    unittest.main()
