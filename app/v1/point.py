import numpy as np

from constants import DEFAULT_FORCE, DEFAULT_VELOCITY


class PointMass:
    def __init__(self, x, y, mass : float=1.0):
        self.mass = mass
        self.position = np.array([x, y], dtype=float)
        self.velocity = DEFAULT_VELOCITY.copy()
        self.force = DEFAULT_FORCE.copy()


class Spring:
    def __init__(self, pm_1 : PointMass, pm_2 : PointMass, k=20):
        self.p1 = pm_1
        self.p2 = pm_2
        self.k = k
        self.length = np.linalg.norm(pm_1.position - pm_2.position)