import pygame
import math
from typing import List

# Initialize Pygame
pygame.init()

# Screen dimensions
WIDTH, HEIGHT = 800, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Soft Body Physics Engine")

# Define colors for drawing
WHITE = (255, 255, 255)
RED = (255, 0, 0)
BLUE = (0, 0, 255)

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
    def __init__(self, gravity: Vec2 = Vec2(0.0, 0.1), spring_force: float = 1.0, spring_damping: float = 0.1):
        self.points: List[PointMass] = []
        self.gravity = gravity
        self.spring_force = spring_force
        self.spring_damping = spring_damping
        self.dragging = None

    def rotate(self, vector: Vec2, angle: float) -> Vec2:
        """Rotates a vector by the given angle (in radians)."""
        return Vec2(vector.x * math.cos(angle) - vector.y * math.sin(angle),
                    vector.x * math.sin(angle) + vector.y * math.cos(angle))

    def update(self, dt: float):
        # Apply gravity to all points that are not being dragged
        for point in self.points:
            if self.dragging != point:
                point.velocity += self.gravity * dt

        # Handle collisions and bounce
        for i, point1 in enumerate(self.points):
            for point2 in self.points[i + 1:]:
                # Check if points are colliding (based on distance)
                delta = point1.position - point2.position
                distance = delta.length()
                if distance < 10:  # Assume a radius of 10 for simplicity
                    normal = delta.normalize()
                    relative_velocity = point1.velocity - point2.velocity
                    velocity_along_normal = relative_velocity.dot(normal)

                    # Elastic collision response (bounce)
                    if velocity_along_normal < 0:  # Only bounce if moving towards each other
                        restitution = 1.0  # Elastic collision
                        impulse = (2 * velocity_along_normal) / (point1.mass + point2.mass)
                        point1.velocity -= normal * impulse * point2.mass
                        point2.velocity += normal * impulse * point1.mass

        # Update positions based on velocities
        for point in self.points:
            point.position += point.velocity * dt

    def draw(self, screen):
        # Draw points as circles
        for point in self.points:
            pygame.draw.circle(screen, RED, point.position.to_tuple(), 10)

def main():
    engine = Engine(spring_force=2.0)

    # Add multiple points (dynamic objects)
    engine.points.append(PointMass(position=Vec2(100.0, 100.0), velocity=Vec2(0.0, 0.0)))
    engine.points.append(PointMass(position=Vec2(200.0, 100.0), velocity=Vec2(0.0, 0.0)))
    engine.points.append(PointMass(position=Vec2(300.0, 100.0), velocity=Vec2(0.0, 0.0)))
    engine.points.append(PointMass(position=Vec2(400.0, 100.0), velocity=Vec2(0.0, 0.0)))

    clock = pygame.time.Clock()
    running = True

    # Game loop
    while running:
        screen.fill(WHITE)

        # Handle events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.MOUSEBUTTONDOWN:
                # Check if mouse clicks on any point mass
                mouse_pos = Vec2(*pygame.mouse.get_pos())
                for point in engine.points:
                    if (point.position - mouse_pos).length() < 10:  # 10 is the click radius
                        engine.dragging = point  # Start dragging the point
                        break

            elif event.type == pygame.MOUSEMOTION:
                if engine.dragging:
                    # Update the dragged point's position
                    mouse_pos = Vec2(*pygame.mouse.get_pos())
                    engine.dragging.position = mouse_pos

            elif event.type == pygame.MOUSEBUTTONUP:
                engine.dragging = None  # Stop dragging

        # Update the physics engine
        engine.update(0.016)  # Assuming 60 FPS (dt = 1/60)

        # Draw the points and interactions
        engine.draw(screen)

        # Refresh the screen
        pygame.display.flip()

        # Limit the frame rate to 60 FPS
        clock.tick(6000)

    pygame.quit()

if __name__ == "__main__":
    main()