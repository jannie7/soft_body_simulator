import numpy as np

from point import Spring


def getSquareCornerPoints(x, y, size):
    return [
        ((x - size), (y + size)), # top left square corner
        ((x + size), (y + size)), # top right square corner
        ((x + size), (y - size)), # bottom right square corner
        ((x - size), (y - size)), # bottom left square corner
    ]

def getCirclePoints(x, y, radius, count):
    dangle = 2 * np.pi / count
    points = []
    
    for i in range(1, count+1):
        angle = i * dangle
        px = x + radius * np.cos(angle)
        py = y + radius * np.sin(angle)
        points.append((px, py))

    return points

def getCircumferenceSprings(points, count):
    springs = []
    for i in range(count):
        next_index = (i + 1) % count
        spring = Spring(points[i], points[next_index])
        springs.append(spring)
    
    return springs
