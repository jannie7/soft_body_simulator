import numpy as np
from pygame.locals import *
import pygame as game

from enums import DISPLAY_DIMENSION
from constants import WINDOW_LABEL, DAMPING, DELTA, BACKGROUND_COLOR, SHAPE_COLOR
from shape import Square, Triangle, Circle


def run():
    game.init()
    surf = game.display.set_mode(DISPLAY_DIMENSION.HD.value)
    game.display.set_caption(WINDOW_LABEL)


    world = []
    world.append(Square(100, 100, 50))
    world.append(Triangle(200, 200, 50))
    world.append(Circle(300, 300, 60, 9))
    world.append(Circle(300, 300, 60, 4))

    selected_point = None
    mouse_pos = None


    while True:
        for event in game.event.get():
            if event.type == game.QUIT:
                game.quit()
                quit()

            if event.type == game.MOUSEBUTTONDOWN:
                x, y = game.mouse.get_pos()
                selected_point = None
                min_dist = float('inf')

                for shape in world:
                    for pm in shape.pm_structure:
                        dist = np.linalg.norm(pm.position - np.array([x, y]))
                        if dist < min_dist:
                            min_dist = dist
                            selected_point = pm

                if selected_point and min_dist < 50:
                    mouse_pos = np.array([x, y], dtype=float)
                else:
                    selected_point = None

            if event.type == game.MOUSEBUTTONUP:
                print("Mouse up")
                selected_point = None

        if selected_point and type(mouse_pos) != type(None):
            x, y = game.mouse.get_pos()
            mouse_pos = np.array([x, y], dtype=float)
            selected_point.position = mouse_pos
            selected_point.velocity = np.array([0,0], dtype=float)
            selected_point.force = np.array([0,0], dtype=float)


        surf.fill(BACKGROUND_COLOR)
        for shape in world:
            for spring in shape.spring_structure:
                dis = spring.p1.position - spring.p2.position
                new_length = np.linalg.norm(dis)
                dir =   dis / new_length
                spring.force = -spring.k * (new_length - spring.length) * dir
                damping_force = -DAMPING * np.sum((spring.p1.velocity - spring.p2.velocity) * dir) * dir
                spring.p1.force += spring.force + damping_force
                spring.p2.force -= spring.force + damping_force

            for pm in shape.pm_structure:
                pm.force += np.array([0,pm.mass * 9.81])
                pm.velocity += pm.force / pm.mass * DELTA
                pm.position += pm.velocity * DELTA
                pm.force *= 0.0
                if pm.position[1] > 600:
                    pm.position[1] = 600
                    pm.velocity[1] *= -1 * 0.5
                    pm.velocity[0] *= 0.1


            for spring in shape.spring_structure:
                i, j = spring.p1.position, spring.p2.position
                game.draw.aaline(surf, SHAPE_COLOR, i, j,4)


        game.display.update()




if __name__ == "__main__":
    run()

