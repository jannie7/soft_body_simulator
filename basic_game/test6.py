import pygame
import math
from typing import List

# Initialize Pygame
pygame.init()

# Screen dimensions
WIDTH, HEIGHT = 800, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Shape Drawing Physics Engine")

# Define colors for drawing
WHITE = (255, 255, 255)
RED = (255, 0, 0)
BLUE = (0, 0, 255)
GREEN = (0, 255, 0)

class Vec2:
    def __init__(self, x: float = 0.0, y: float = 0.0):
        self.x = x
        self.y = y

    def __add__(self, other):
        return Vec2(self.x + other.x, self.y + other.y)

    def __mul__(self, scalar: float):
        return Vec2(self.x * scalar, self.y * scalar)

    def __iadd__(self, other):
        self.x += other.x
        self.y += other.y
        return self

    def __sub__(self, other):
        return Vec2(self.x - other.x, self.y - other.y)

    def __truediv__(self, scalar: float):
        return Vec2(self.x / scalar, self.y / scalar)

    def dot(self, other):
        return self.x * other.x + self.y * other.y

    def cross(self, other):
        return self.x * other.y - self.y * other.x

    def length(self):
        return math.sqrt(self.x ** 2 + self.y ** 2)

    def normalize(self):
        length = self.length()
        return self if length == 0 else self / length

    def __repr__(self):
        return f"Vec2(x={self.x}, y={self.y})"

    def to_tuple(self):
        return int(self.x), int(self.y)

class PointMass:
    def __init__(self, position: Vec2 = Vec2(), velocity: Vec2 = Vec2(), mass: float = 1.0):
        self.position = position
        self.velocity = velocity
        self.mass = mass  # Mass for collision response

class Engine:
    def __init__(self, spring_force: float = 1.0, spring_damping: float = 0.1):
        self.points: List[PointMass] = []
        self.lines: List[tuple] = []  # List to store lines (pairs of indices)
        self.spring_force = spring_force
        self.spring_damping = spring_damping
        self.dragging = None
        self.selected_shape = None  # Can be 'triangle' or 'square'
        self.max_points = 0  # 3 for triangle, 4 for square

    def update(self, dt: float):
        # Handle physics if needed (currently no gravity)
        pass

    def draw(self, screen):
        # Draw points as circles
        for point in self.points:
            pygame.draw.circle(screen, RED, point.position.to_tuple(), 10)

        # Draw lines between points that are in the 'lines' list
        for line in self.lines:
            point1, point2 = line
            p0 = self.points[point1].position
            p1 = self.points[point2].position
            pygame.draw.line(screen, GREEN, p0.to_tuple(), p1.to_tuple(), 2)

    def reset(self):
        # Reset points and lines after drawing a shape
        self.points.clear()
        self.lines.clear()
        self.selected_shape = None
        self.max_points = 0

def main():
    engine = Engine(spring_force=2.0)

    clock = pygame.time.Clock()
    running = True
    dragging = None  # Store the index of the dragged point
    clicked_point = None  # Store the first clicked point to create a line

    # Game loop
    while running:
        screen.fill(WHITE)

        # Handle events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.MOUSEBUTTONDOWN:
                mouse_pos = Vec2(*pygame.mouse.get_pos())
                
                if engine.selected_shape == 'triangle' and len(engine.points) < 3:
                    # Add points for triangle
                    engine.points.append(PointMass(position=mouse_pos))
                    if len(engine.points) == 3:
                        # Connect the points to form a triangle
                        engine.lines.append((0, 1))
                        engine.lines.append((1, 2))
                        engine.lines.append((2, 0))
                
                elif engine.selected_shape == 'square' and len(engine.points) < 4:
                    # Add points for square
                    engine.points.append(PointMass(position=mouse_pos))
                    if len(engine.points) == 4:
                        # Connect the points to form a square
                        engine.lines.append((0, 1))
                        engine.lines.append((1, 2))
                        engine.lines.append((2, 3))
                        engine.lines.append((3, 0))

                if len(engine.points) == engine.max_points:
                    engine.selected_shape = None  # Disable further dragging

            elif event.type == pygame.MOUSEMOTION:
                if dragging is not None:
                    # Update the dragged point's position
                    mouse_pos = Vec2(*pygame.mouse.get_pos())
                    engine.points[dragging].position = mouse_pos

            elif event.type == pygame.MOUSEBUTTONUP:
                dragging = None  # Stop dragging

            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_t:  # Press 'T' for Triangle
                    engine.reset()  # Reset the engine for a new shape
                    engine.selected_shape = 'triangle'
                    engine.max_points = 3
                elif event.key == pygame.K_s:  # Press 'S' for Square
                    engine.reset()  # Reset the engine for a new shape
                    engine.selected_shape = 'square'
                    engine.max_points = 4

        # Update the physics engine
        engine.update(0.016)  # Assuming 60 FPS (dt = 1/60)

        # Draw the points, lines, and interactions
        engine.draw(screen)

        # Refresh the screen
        pygame.display.flip()

        # Limit the frame rate to 60 FPS
        clock.tick(60)

    pygame.quit()

if __name__ == "__main__":
    main()