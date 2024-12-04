import numpy as np

from enums import COLORS

DELTA = 0.01
DAMPING = 2
WINDOW_LABEL = "Soft Body Physics V1"

SQAURE_CONNECTIONS = [
    (0, 1), # spring connecting top horizontal points
    (1, 2), # spring connecting right vertical points
    (2, 3), # spring connecting bottom horizontal points
    (3, 0), # spring connecting left vertical points
    (0, 2), # spring connecting primary diagonal
    (1, 3), # spring connecting secondary diagonal
]
SQUARE_SIDES = 4

DEFAULT_VELOCITY = np.array([0.0, 0.0], dtype=float)
DEFAULT_FORCE = np.array([0.0, 0.0], dtype=float)

BACKGROUND_COLOR = COLORS.WHITE.value
SHAPE_COLOR = COLORS.BLACK.value
