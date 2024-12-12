
import numpy as np

from constants import SQAURE_CONNECTIONS, SQUARE_SIDES
from utils import getSquareCornerPoints, getCirclePoints, getCircumferenceSprings
from point import PointMass, Spring


class Shape:
    def __init__(self):
        self.pm_structure = []
        self.spring_structure = []
        self.outedge = []


class Square(Shape):
    def __init__(self, x, y, size):
        super().__init__()
        self.pm_structure = [PointMass(i, j) for i, j in getSquareCornerPoints(x, y, size)]
        self.spring_structure = [Spring(self.pm_structure[i], self.pm_structure[j]) for i, j in SQAURE_CONNECTIONS]
        self.outedge = [self.spring_structure[i] for i in range(SQUARE_SIDES)]


class Circle(Shape):
    def __init__(self, x, y, radius, count=20):
        super().__init__()
        self.pm_structure = [PointMass(x, y)] + [PointMass(i, j) for i, j in getCirclePoints(x, y, radius, count)]
        self.spring_structure = getCircumferenceSprings(self.pm_structure[1:], count)
        self.outedge = self.spring_structure.copy()
        for i in range(1, len(self.pm_structure)):
            self.spring_structure.append(Spring(self.pm_structure[0], self.pm_structure[i]))

        self.spring_structure.append(Spring(self.pm_structure[1], self.pm_structure[-1]))
        self.outedge.append(self.spring_structure[-1])



class Triangle(Circle):
    def __init__(self, x, y, size):
        size = 2 * size / (np.sqrt(3))
        super().__init__(x, y, size, 3)
