import pygame
import math

# Pygame setup
pygame.init()

# Window dimensions
WIDTH, HEIGHT = 800, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Soft Body Physics with Multiple Shapes")

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)

# Particle (Node) class
class Particle:
    def __init__(self, x, y, radius=5, mass=1):
        self.x = x
        self.y = y
        self.radius = radius
        self.mass = mass
        self.vx = 0
        self.vy = 0

    def apply_force(self, fx, fy):
        self.vx += fx / self.mass
        self.vy += fy / self.mass

    def update(self, dt):
        # Update position based on velocity
        self.x += self.vx * dt
        self.y += self.vy * dt

    def draw(self, screen):
        pygame.draw.circle(screen, RED, (int(self.x), int(self.y)), self.radius)

# Spring class to represent the spring force between particles
class Spring:
    def __init__(self, p1, p2, rest_length, k=0.1):
        self.p1 = p1
        self.p2 = p2
        self.rest_length = rest_length  # Ideal distance between particles
        self.k = k  # Spring constant

    def apply_force(self):
        dx = self.p2.x - self.p1.x
        dy = self.p2.y - self.p1.y
        dist = math.sqrt(dx * dx + dy * dy)
        force_magnitude = (dist - self.rest_length) * self.k
        fx = (dx / dist) * force_magnitude
        fy = (dy / dist) * force_magnitude
        self.p1.apply_force(fx, fy)
        self.p2.apply_force(-fx, -fy)

# Function to handle mouse interaction
def attract_to_mouse(particle, mouse_pos, strength=0.1):
    mx, my = mouse_pos
    dx = mx - particle.x
    dy = my - particle.y
    dist = math.sqrt(dx * dx + dy * dy)
    if dist > 1:
        force_x = dx / dist * strength
        force_y = dy / dist * strength
        particle.apply_force(force_x, force_y)

# Function to handle boundary collision
def handle_boundaries(particle):
    # Prevent particle from moving out of bounds
    if particle.x < 0:
        particle.x = 0
        particle.vx = 0
    elif particle.x > WIDTH:
        particle.x = WIDTH
        particle.vx = 0

    if particle.y < 0:
        particle.y = 0
        particle.vy = 0
    elif particle.y > HEIGHT:
        particle.y = HEIGHT
        particle.vy = 0

# Function to create a shape (triangle, square, or circle)
def create_shape(shape, center_x, center_y, size):
    particles = []
    springs = []

    if shape == 'triangle':
        # Triangle (equilateral)
        p1 = Particle(center_x, center_y - size)
        p2 = Particle(center_x - size * math.sqrt(3) / 2, center_y + size / 2)
        p3 = Particle(center_x + size * math.sqrt(3) / 2, center_y + size / 2)
        
        particles = [p1, p2, p3]
        springs = [
            Spring(p1, p2, size),
            Spring(p2, p3, size),
            Spring(p3, p1, size)
        ]
        
    elif shape == 'square':
        # Square (4 particles)
        p1 = Particle(center_x - size / 2, center_y - size / 2)
        p2 = Particle(center_x + size / 2, center_y - size / 2)
        p3 = Particle(center_x + size / 2, center_y + size / 2)
        p4 = Particle(center_x - size / 2, center_y + size / 2)
        
        particles = [p1, p2, p3, p4]
        springs = [
            Spring(p1, p2, size),
            Spring(p2, p3, size),
            Spring(p3, p4, size),
            Spring(p4, p1, size),
            Spring(p1, p3, math.sqrt(2) * size),  # Diagonal spring
            Spring(p2, p4, math.sqrt(2) * size)   # Diagonal spring
        ]
        
    elif shape == 'circle':
        # Circle (8 particles, forming a polygon)
        particles = []
        angle_step = math.pi * 2 / 8
        for i in range(8):
            x = center_x + size * math.cos(i * angle_step)
            y = center_y + size * math.sin(i * angle_step)
            particles.append(Particle(x, y))
        
        for i in range(8):
            next_i = (i + 1) % 8
            springs.append(Spring(particles[i], particles[next_i], size))

    return particles, springs

# Main simulation loop
def run_simulation():
    clock = pygame.time.Clock()

    # Create multiple objects (shapes)
    shapes = [
        {'shape': 'triangle', 'center_x': 200, 'center_y': 200, 'size': 100},
        {'shape': 'square', 'center_x': 500, 'center_y': 150, 'size': 80},
        {'shape': 'circle', 'center_x': 600, 'center_y': 400, 'size': 50},
        {'shape': 'square', 'center_x': 300, 'center_y': 450, 'size': 60}
    ]
    
    # Create list of particles and springs for all shapes
    all_particles = []
    all_springs = []
    for shape in shapes:
        particles, springs = create_shape(shape['shape'], shape['center_x'], shape['center_y'], shape['size'])
        all_particles.append(particles)
        all_springs.append(springs)

    # Simulation loop
    running = True
    while running:
        screen.fill(WHITE)

        dt = clock.tick(60) / 1000.0  # Delta time

        # Check for quit event
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Get the mouse position
        mouse_pos = pygame.mouse.get_pos()

        # Apply forces and update particles for all objects
        for i, particles in enumerate(all_particles):
            springs = all_springs[i]

            # Apply spring forces
            for spring in springs:
                spring.apply_force()

            # Apply mouse attraction and update particles
            for particle in particles:
                attract_to_mouse(particle, mouse_pos)  # Attract particles to the mouse
                particle.update(dt)  # Update particle position
                handle_boundaries(particle)  # Prevent out of bounds movement
                particle.draw(screen)  # Draw particle

            # Draw springs (lines between particles)
            for spring in springs:
                pygame.draw.line(screen, BLACK, (int(spring.p1.x), int(spring.p1.y)), (int(spring.p2.x), int(spring.p2.y)), 2)

        pygame.display.flip()

    pygame.quit()

# Run the simulation
run_simulation()
