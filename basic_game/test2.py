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
    def __init__(self, position: Vec2 = Vec2(), velocity: Vec2 = Vec2()):
        self.position = position
        self.velocity = velocity

class SoftBody:
    class Vertex:
        def __init__(self, index: int, position: Vec2 = Vec2()):
            self.index = index
            self.position = position

    def __init__(self):
        self.vertices: List[SoftBody.Vertex] = []

class Engine:
    def __init__(self, gravity: Vec2 = Vec2(), spring_force: float = 1.0, spring_damping: float = 0.1):
        self.points: List[PointMass] = []
        self.gravity = gravity
        self.spring_force = spring_force
        self.spring_damping = spring_damping
        self.soft_bodies: List[SoftBody] = []

    def rotate(self, vector: Vec2, angle: float) -> Vec2:
        """Rotates a vector by the given angle (in radians)."""
        return Vec2(vector.x * math.cos(angle) - vector.y * math.sin(angle),
                    vector.x * math.sin(angle) + vector.y * math.cos(angle))

    def update(self, dt: float):
        # Apply gravity to points
        for point in self.points:
            point.velocity += self.gravity * dt

        # Soft body dynamics
        for body in self.soft_bodies:
            # Compute the center of mass
            center = Vec2(0.0, 0.0)
            for v in body.vertices:
                center += self.points[v.index].position
            center /= len(body.vertices)

            # Compute the shape's rotation angle
            A = 0.0
            B = 0.0
            for v in body.vertices:
                r = self.points[v.index].position - center
                A += r.dot(v.position)
                B += r.cross(v.position)

            angle = -math.atan2(B, A)

            # Apply spring forces to vertices
            for v in body.vertices:
                target = center + self.rotate(v.position, angle)
                delta = target - self.points[v.index].position
                self.points[v.index].velocity += delta * dt * self.spring_force

    def draw(self, screen):
        # Draw soft body vertices
        for body in self.soft_bodies:
            for v in body.vertices:
                pygame.draw.circle(screen, RED, self.points[v.index].position.to_tuple(), 5)

            # Optionally, draw lines between vertices to visualize the soft body edges
            for i in range(len(body.vertices)):
                for j in range(i + 1, len(body.vertices)):
                    p0 = self.points[body.vertices[i].index].position
                    p1 = self.points[body.vertices[j].index].position
                    pygame.draw.line(screen, BLUE, p0.to_tuple(), p1.to_tuple(), 2)

def main():
    engine = Engine(spring_force=2.0)

    # Add points
    engine.points.append(PointMass(position=Vec2(100.0, 100.0), velocity=Vec2(0.0, 0.0)))
    engine.points.append(PointMass(position=Vec2(200.0, 100.0), velocity=Vec2(0.0, 0.0)))
    engine.points.append(PointMass(position=Vec2(200.0, 200.0), velocity=Vec2(0.0, 0.0)))
    engine.points.append(PointMass(position=Vec2(100.0, 200.0), velocity=Vec2(0.0, 0.0)))

    # Create a soft body and add it
    soft_body = SoftBody()
    soft_body.vertices.append(SoftBody.Vertex(index=0, position=Vec2(100.0, 100.0)))
    soft_body.vertices.append(SoftBody.Vertex(index=1, position=Vec2(200.0, 100.0)))
    soft_body.vertices.append(SoftBody.Vertex(index=2, position=Vec2(200.0, 200.0)))
    soft_body.vertices.append(SoftBody.Vertex(index=3, position=Vec2(100.0, 200.0)))
    engine.soft_bodies.append(soft_body)

    clock = pygame.time.Clock()
    running = True

    # Game loop
    while running:
        screen.fill(WHITE)

        # Handle events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Update the physics engine
        engine.update(0.016)  # Assuming 60 FPS (dt = 1/60)

        # Draw the soft body
        engine.draw(screen)

        # Refresh the screen
        pygame.display.flip()

        # Limit the frame rate to 60 FPS
        clock.tick(60)

    pygame.quit()

if __name__ == "__main__":
    main()