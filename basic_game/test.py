
from typing import List
import math

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



class Collision:
    def __init__(self, normal: Vec2 = Vec2(0.0, 0.0), depth: float = -math.inf):
        self.normal = normal
        self.depth = depth

    def __repr__(self):
        return f"Collision(normal={self.normal}, depth={self.depth})"


class Planet:
    def __init__(self, center: Vec2 = Vec2(), radius: float = 0.0):
        self.center = center
        self.radius = radius

    def __repr__(self):
        return f"Planet(center={self.center}, radius={self.radius})"


class DistanceConstraint:
    def __init__(self, index0: int, index1: int, distance: float):
        self.index0 = index0
        self.index1 = index1
        self.distance = distance


class Engine:
    def __init__(self, gravity: Vec2 = Vec2(), spring_force: float = 1.0, elasticity: float = 0.5, friction: float = 0.1, constraint_damping: float = 0.1, spring_damping: float = 0.1):
        self.points: List[PointMass] = []
        self.constraints: List[DistanceConstraint] = []
        self.gravity = gravity
        self.elasticity = elasticity
        self.friction = friction
        self.constraint_damping = constraint_damping
        self.the_world = []  # List of planets or collision objects
        self.spring_force = spring_force
        self.spring_damping = spring_damping
        self.soft_bodies: List[SoftBody] = []


    def rotate(self, vector: Vec2, angle: float) -> Vec2:
        """Rotates a vector by the given angle (in radians)."""
        return Vec2(vector.x * math.cos(angle) - vector.y * math.sin(angle),
                    vector.x * math.sin(angle) + vector.y * math.cos(angle))


    def find_collision(self, position: Vec2, the_world) -> Collision:
        result = Collision()
        for obj in the_world:
            collision = self.find_collision_with_planet(position, obj)
            if collision.depth > result.depth:
                result = collision
        return result

    def find_collision_with_planet(self, position: Vec2, planet) -> Collision:
        delta = position - planet.center
        distance = delta.length()
        if distance == 0:
            normal = Vec2(0, 1)  # Avoid division by zero
        else:
            normal = delta / distance
        depth = planet.radius - distance
        return Collision(normal=normal, depth=depth)

    def update(self, dt: float):
        # Update velocities (gravity or other forces can be applied here)
        for point in self.points:
            point.velocity += self.gravity * dt

        # Check for collisions and resolve
        for point in self.points:
            collision = self.find_collision(point.position, self.the_world)

            # Check if collision took place
            if collision.depth < 0.0:
                continue

            # Resolve the constraint
            point.position += collision.normal * collision.depth

            # Compute normal and tangential velocity
            vn = collision.normal * point.velocity.dot(collision.normal)
            vt = point.velocity - vn

            # Apply bouncing (elasticity)
            vn = collision.normal * (-self.elasticity * vn.dot(collision.normal))

            # Apply friction
            vt *= math.exp(-self.friction * dt)

            # Update velocity
            point.velocity = vn + vt

        # Distance constraint resolution
        for constraint in self.constraints:
            p0 = self.points[constraint.index0].position
            p1 = self.points[constraint.index1].position

            delta = p1 - p0
            distance = delta.length()
            direction = delta / distance

            if distance == 0:
                continue  # Avoid division by zero

            required_delta = delta * (constraint.distance / distance)
            damping_factor = 1.0 - math.exp(-self.constraint_damping * dt)
            offset = (required_delta - delta) * damping_factor

            required_delta = direction * constraint.distance
            force = self.spring_force * (required_delta - delta)

            # force = delta - required_delta  # Force is proportional to deviation
            # force *= self.spring_force  # Scale by spring force constant

            p0.velocity -= force * dt
            p1.velocity += force * dt

            self.points[constraint.index0].position -= offset / 2.0
            self.points[constraint.index1].position += offset / 2.0

            # Damping
            vrel = (p1.velocity - p0.velocity).dot(direction)
            damping_factor = math.exp(-self.spring_damping * dt)
            new_vrel = vrel * damping_factor
            vrel_delta = new_vrel - vrel

            p0.velocity -= direction * (vrel_delta / 2.0)
            p1.velocity += direction * (vrel_delta / 2.0)


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

            # Apply spring forces
            for v in body.vertices:
                target = center + self.rotate(v.position, angle)
                delta = target - self.points[v.index].position
                self.points[v.index].velocity += delta * dt * self.spring_force




def find_collision(position: Vec2) -> Collision:
    return Collision(normal=Vec2(0.0, 1.0), depth=-position.y)


def find_collision(position: Vec2, planet: Planet) -> Collision:
    delta = position - planet.center
    distance = delta.length()
    normal = delta / distance
    depth = planet.radius - distance
    return Collision(normal=normal, depth=depth)

def find_collision_with_planet(position: Vec2, planet: Planet) -> Collision:
    delta = position - planet.center
    distance = delta.length()
    if distance == 0:
        # Avoid division by zero
        normal = Vec2(0, 1)
    else:
        normal = delta / distance
    depth = planet.radius - distance
    return Collision(normal=normal, depth=depth)


def find_collision(position: Vec2, planets: List[Planet]) -> Collision:
    result = Collision()
    for planet in planets:
        collision = find_collision_with_planet(position, planet)
        if collision.depth > result.depth:
            result = collision
    return result






# Example usage
if __name__ == "__main__":
    engine = Engine(spring_force=2.0)

    # Add points
    engine.points.append(PointMass(position=Vec2(0.0, 0.0), velocity=Vec2(0.0, 0.0)))
    engine.points.append(PointMass(position=Vec2(3.0, 0.0), velocity=Vec2(0.0, 0.0)))
    engine.points.append(PointMass(position=Vec2(3.0, 3.0), velocity=Vec2(0.0, 0.0)))
    engine.points.append(PointMass(position=Vec2(0.0, 3.0), velocity=Vec2(0.0, 0.0)))

    # Create a soft body and add it
    soft_body = SoftBody()
    soft_body.vertices.append(SoftBody.Vertex(index=0, position=Vec2(0.0, 0.0)))
    soft_body.vertices.append(SoftBody.Vertex(index=1, position=Vec2(3.0, 0.0)))
    soft_body.vertices.append(SoftBody.Vertex(index=2, position=Vec2(3.0, 3.0)))
    soft_body.vertices.append(SoftBody.Vertex(index=3, position=Vec2(0.0, 3.0)))
    engine.soft_bodies.append(soft_body)

    # Update the engine
    print("Before update:")
    for i, point in enumerate(engine.points):
        print(f"Point {i}: Position={point.position}, Velocity={point.velocity}")

    engine.update(0.1)

    print("\nAfter update:")
    for i, point in enumerate(engine.points):
        print(f"Point {i}: Position={point.position}, Velocity={point.velocity}")






