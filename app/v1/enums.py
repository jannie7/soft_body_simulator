from enum import Enum

import pygame as game


class DISPLAY_DIMENSION(Enum):
    HD = (1280, 720)
    FULL_HD = (1920, 1080)
    TWO_K = (2560, 1440)
    FOUR_K = (3840, 2160) 


class COLORS(Enum):
    RED = game.Color(255, 0, 0)
    GREEN = game.Color(0, 255, 0)
    BLUE = game.Color(0, 0, 255)
    YELLOW = game.Color(255,255,0)
    WHITE = game.Color(255, 255, 255)
    BLACK = game.Color(0, 0, 0)

