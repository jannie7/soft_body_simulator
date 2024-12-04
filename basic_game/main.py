import math
import time

import pygame as game
from pygame import gfxdraw
from pygame.locals import *

from utils import DISPLAY_DIMENSION, COLORS
from constants import WINDOW_LABEL






def main():
    game.init()
    surf = game.display.set_mode(DISPLAY_DIMENSION.HD.value, game.RESIZABLE)
    game.display.set_caption(WINDOW_LABEL)

    while True:
        for event in game.event.get():
            if event.type == game.QUIT:
                game.quit()
                quit()
        





if __name__ == "__main__":
    print("Hello World!")
    main()


