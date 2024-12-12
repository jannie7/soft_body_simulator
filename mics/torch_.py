import pygame as pg
from pygame import gfxdraw
import math
from pygame.locals import *

import torch
import matplotlib.pyplot as plt
import time

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
# device = torch.device("cpu")

SPRING_CONSTANT = 10
DAMPING = 2
SIM_SPEED = 2

GRAVITY = torch.tensor([0, 9.8], device=device)




def add_square(positions, springs, top_left, size, connect=False, type='right'):
    num_points = positions.shape[0]
    new_positions = torch.tensor([
        top_left,
        [top_left[0] + size, top_left[1]],
        [top_left[0] + size, top_left[1] + size],
        [top_left[0], top_left[1] + size]
    ], device=device)
    positions = torch.cat((positions, new_positions), dim=0)

    new_springs = torch.tensor([
        [num_points, num_points + 1],
        [num_points + 1, num_points + 2],
        [num_points + 2, num_points + 3],
        [num_points + 3, num_points],
        [num_points, num_points + 2],
        [num_points + 1, num_points + 3]
    ], device=device)
    springs = torch.cat((springs, new_springs), dim=0)

    # Connect the new square with the last square
    if num_points > 0 and connect:
        connections = [-2, -3, 0, 3]
        if type == 'down':
            connections = [0, 1, -2, -1]
        elif type == 'up':
            connections = [3, 2, -3, -4]
        elif type == 'left':
            connections = [2, 1, -4, -1]

        new_connections = torch.tensor([
            [num_points + connections[1], num_points + connections[2]],
            [num_points + connections[0], num_points + connections[3]],
            [num_points + connections[1], num_points + connections[3]],
            [num_points + connections[0], num_points + connections[2]],
        ], device=device)

        springs = torch.cat((springs, new_connections), dim=0)

    return positions, springs

# Initialize positions of points in a square
positions = torch.tensor([
    [0, 0.0],
    [50, 0.0],
    [50, 50],
    [0, 50]
], device=device)
# Define the springs as connections (pairs of indices)
springs = torch.tensor([
    [0, 1],  # Spring between point 0 and 1
    [1, 2],  # Spring between point 1 and 2
    [2, 3],  # Spring between point 2 and 3
    [3, 0],  # Spring between point 3 and 0
    [0, 2],  # Diagonal spring
    [1, 3],
], device=device)

positions, springs = add_square(positions, springs, [90, 260], 50)
positions, springs = add_square(positions, springs, [90, 350], 50, connect=True, type='down')
positions, springs = add_square(positions, springs, [190, 350], 50, connect=True, type='right')
positions, springs = add_square(positions, springs, [290, 350], 50, connect=True, type='right')
positions, springs = add_square(positions, springs, [290, 250], 50, connect=True, type='up')
positions, springs = add_square(positions, springs, [390, 250], 50, connect=True, type='right')
positions, springs = add_square(positions, springs, [390, 320], 50, connect=True, type='down')
# for i in range(1000):
#     positions, springs = add_square(positions, springs, [50*i % 1280, 0], 50)


lengths = torch.ones(springs.shape[0], device=device)

for s, spring in enumerate(springs):
    i, j = spring
    displacement = positions[j] - positions[i]
    length = torch.norm(displacement)
    lengths[s] = length

print(positions)

# Mass of each point
masses = torch.ones(positions.shape[0], device=device) * 0.2
# Initialize velocities
velocities = torch.zeros_like(positions, device=device)

forces = torch.zeros_like(positions, device=device)

print(springs[:, 1])
print(springs[:, 0])
print(positions[springs[:, 1]])
print(positions[springs[:, 0]])

# Simulation loop
def simulate(delta):
    delta *= SIM_SPEED
    
    global positions, velocities, forces

    forces.fill_(0)

    # Apply gravity to all points
    forces += GRAVITY * masses[:, None]

    # Spring forces
    displacement = positions[springs[:, 1]] - positions[springs[:, 0]]
    length = torch.norm(displacement, dim=1)
    direction = displacement / length[:, None]  # Normalize displacement

    spring_force = SPRING_CONSTANT * (length - lengths)[:, None] * direction

    relative_velocity = velocities[springs[:, 1]] - velocities[springs[:, 0]]
    damping_force = DAMPING * torch.sum(relative_velocity * direction, dim=1)[:, None] * direction

    total_force = spring_force + damping_force

    forces.index_add_(0, springs[:, 0], total_force)
    forces.index_add_(0, springs[:, 1], -total_force)

    accelerations = forces / masses[:, None]
    velocities += accelerations * delta
    positions += velocities * delta

    below_ground = positions[:, 1] > 600
    positions[below_ground, 1] = 600    
    velocities[below_ground, 1] *= -0.5
    

# Visualization function
def visualize(surf):
    for s, spring in enumerate(springs):
        i, j = spring
        pg.draw.aaline(surf, (255, 255, 255), positions[i].tolist(), positions[j].tolist(), 4)

dt = 0.03

def main():
    global dt
    pg.init()
    display = (1280, 720)

    surf = pg.display.set_mode(display,pg.RESIZABLE)
    pg.display.set_caption("Test")

    while True:
        for event in pg.event.get():
            if event.type == pg.QUIT:
                pg.quit()
                quit()
        start = time.perf_counter()
        
        simulate(dt)
        time_viz = time.perf_counter()
        surf.fill((0, 0, 0, 0))
        visualize(surf)
        time_viz = time.perf_counter() - time_viz
        pg.draw.line(surf, (255, 255, 255), (0, 600), (1280, 600), 2)
        pg.display.update()
        dt = (time.perf_counter() - start)
        # dt = max(min(dt,0.01), 0.001)
        print(f"dt: {dt*1000:2f}ms fps: {1/dt:2f} time_viz: {time_viz:2f}ms")

if __name__ == "__main__":
    main()
