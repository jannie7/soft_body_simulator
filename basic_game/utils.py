from enum import Enum

import pygame as game



class DISPLAY_DIMENSION(Enum):
    FOUR_K = 1
    HD = (1280, 720)


class COLORS(Enum):
    RED = game.Color(255, 0, 0)
    GREEN = game.Color(0, 255, 0)
    BLUE = game.Color(0, 0, 255)
    YELLOW = game.Color(255,255,0)
    WHITE = game.Color(255, 255, 255)

