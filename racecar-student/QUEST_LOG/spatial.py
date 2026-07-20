import numpy as np
import math

class Position:
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y

    def to_pose(self):
        return Pose(self.x, self.y, 0)

    def to_array(self):
        return np.array([self.x, self.y])
    
    def to_absolute(self, reference_pose):
        """Convert this local position into the global frame defined by reference_pose."""
        c = math.cos(reference_pose.dir)
        s = math.sin(reference_pose.dir)

        x = reference_pose.x + self.x * c - self.y * s,
        y = reference_pose.y + self.x * s + self.y * c,

        return Position(x, y)

    def dot(self, other):
        return self.x * other.x + self.y * other.y
    
    def magnitude(self):
        return math.sqrt(self.x*self.x + self.y*self.y)
    
    def round_to_nearest(self, i):
        self.x = self.x//i * i
        self.y = self.y//i * i

    def distance(self, other):
        return (self - other).magnitude()

    # Position + Position
    def __add__(self, other):
        if isinstance(other, Position):
            return Position(self.x + other.x, self.y + other.y)
        return NotImplemented

    # Position - Position
    def __sub__(self, other):
        if isinstance(other, Position):
            return Position(self.x - other.x, self.y - other.y)
        return NotImplemented

    # Position * scalar
    def __mul__(self, scalar):
        if isinstance(scalar, (int, float)):
            return Position(self.x * scalar, self.y * scalar)
        return NotImplemented

    # scalar * Position
    __rmul__ = __mul__

    # Position / scalar
    def __truediv__(self, scalar):
        if isinstance(scalar, (int, float)):
            return Position(self.x / scalar, self.y / scalar)
        return NotImplemented

    def __repr__(self):
        return f"Position(x={self.x}, y={self.y})"


class Pose:
    def __init__(self, x=0, y=0, dir=0):
        self.x = x
        self.y = y
        self.dir = dir

    def to_position(self):
        return Position(self.x, self.y)

    def to_array(self):
        return np.array([self.x, self.y, self.dir])

    # Pose + Pose
    def __add__(self, other):
        if isinstance(other, Pose):
            return Pose(
                self.x + other.x,
                self.y + other.y,
                self.dir + other.dir,
            )
        return NotImplemented

    # Pose - Pose
    def __sub__(self, other):
        if isinstance(other, Pose):
            return Pose(
                self.x - other.x,
                self.y - other.y,
                self.dir - other.dir,
            )
        return NotImplemented

    def __repr__(self):
        return f"Pose(x={self.x}, y={self.y}, dir={self.dir})"