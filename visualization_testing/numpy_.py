import pygame as pg
from pygame import gfxdraw
import math
from pygame.locals import *

import numpy as np
import matplotlib.pyplot as plt
import time


SPRING_CONSTANT = 10
DAMPING = 2
SIM_SPEED = 2

GRAVITY = np.array([0, 9.8])



def add_square(positions, springs, top_left, size,connect=False,type = 'right'):
    num_points = positions.shape[0]
    new_positions = np.array([
        top_left,
        [top_left[0] + size, top_left[1]],
        [top_left[0] + size, top_left[1] + size],
        [top_left[0], top_left[1] + size]
    ])
    positions = np.vstack((positions, new_positions))

    new_springs = np.array([
        [num_points, num_points + 1],
        [num_points + 1, num_points + 2],
        [num_points + 2, num_points + 3],
        [num_points + 3, num_points],
        [num_points, num_points + 2],
        [num_points + 1, num_points + 3]
    ])
    springs = np.vstack((springs, new_springs))

    # Connect the new square with the last square
    if num_points > 0 and connect:   
        connections = [-2,-3,0,3]
        if type == 'down': 
            connections = [0,1,-2,-1]   
        elif type == 'up': 
            connections = [3,2,-3,-4]   
        elif type == 'left':
            connections = [2,1,-4,-1]   
                    
        
        new_connections = np.array([
            [num_points+connections[1], num_points+connections[2]],
            [num_points+connections[0], num_points+connections[3]],
            [num_points+connections[1], num_points+connections[3]],
            [num_points+connections[0], num_points+connections[2]],
        ])
            
        springs = np.vstack((springs, new_connections))

    return positions, springs


# Initialize positions of points in a square
positions = np.array([
    [0, 0.0],
    [50, 0.0],
    [50, 50],
    [0, 50]
])
# Define the springs as connections (pairs of indices)
springs = np.array([
    [0, 1],  # Spring between point 0 and 1
    [1, 2],  # Spring between point 1 and 2
    [2, 3],  # Spring between point 2 and 3
    [3, 0],  # Spring between point 3 and 0
    [0, 2],  # Diagonal spring
    [1, 3],
])


positions, springs = add_square(positions, springs, [90, 260], 50)
positions, springs = add_square(positions, springs, [90, 350], 50,connect=True,type='down')
positions, springs = add_square(positions, springs, [190, 350], 50,connect=True,type='right')
positions, springs = add_square(positions, springs, [290, 350], 50,connect=True,type='right')
positions, springs = add_square(positions, springs, [290, 250], 50,connect=True,type='up')
positions, springs = add_square(positions, springs, [390, 250], 50,connect=True,type='right')
positions, springs = add_square(positions, springs, [390, 320], 50,connect=True,type='down')
positions, springs = add_square(positions, springs, [390+90, 350], 50,connect=True,type='right')
positions, springs = add_square(positions, springs, [390+190, 350], 50,connect=True,type='right')
positions, springs = add_square(positions, springs, [390+290, 350], 50,connect=True,type='right')
positions, springs = add_square(positions, springs, [390+290, 250], 50,connect=True,type='up')
positions, springs = add_square(positions, springs, [390+390, 250], 50,connect=True,type='right')
positions, springs = add_square(positions, springs, [390+390, 320], 50,connect=True,type='down')



# for i in range(1000):
#     positions, springs = add_square(positions, springs, [50*i % 1280, 0], 50)

lengths = np.ones(springs.shape[0])

for s,spring in enumerate(springs):
    i, j = spring
    displacement = positions[j] - positions[i]
    length = np.linalg.norm(displacement)
    lengths[s]=length



print(positions)

# Mass of each point
masses = np.ones(positions.shape[0])*0.2
# Initialize velocities
velocities = np.zeros_like(positions)

forces = np.zeros_like(positions)

# Simulation loop
def simulate(delta):
    delta *= SIM_SPEED
    global positions, velocities,forces
    
    forces.fill(0)    

    # Apply gravity to all points
    forces += GRAVITY * masses[:, np.newaxis]

    # # Spring forces
    # for s,spring in  enumerate(springs):
    #     i, j = spring
    #     displacement = positions[j] - positions[i]
    #     length = np.linalg.norm(displacement)
    #     direction = displacement / length if length != 0 else np.zeros(2)
    #     # Hooke's law force
    #     spring_force = SPRING_CONSTANT * (length - lengths[s]) * direction

    #     # Damping force
    #     relative_velocity = velocities[j] - velocities[i]
    #     damping_force = DAMPING * np.dot(relative_velocity, direction) * direction

    #     # Apply forces
    #     forces[i] += spring_force + damping_force
    #     forces[j] -= spring_force + damping_force
    

    displacement = positions[springs[:, 1]] - positions[springs[:, 0]]  # shape: (num_springs, 2)
    
    lengths_current = np.linalg.norm(displacement, axis=1)  # shape: (num_springs,)

    direction = displacement / lengths_current[:, None]  # shape: (num_springs, 2)

    spring_force = SPRING_CONSTANT * (lengths_current - lengths)[:, None] * direction  # shape: (num_springs, 2)

    relative_velocity = velocities[springs[:, 1]] - velocities[springs[:, 0]]  # shape: (num_springs, 2)

    damping_force = DAMPING * np.sum(relative_velocity * direction, axis=1)[:, None] * direction  # shape: (num_springs, 2)

    total_force = spring_force + damping_force  # shape: (num_springs, 2)
   
    np.add.at(forces, springs[:, 0], total_force)  
    np.add.at(forces, springs[:, 1], -total_force)

    # Update accelerations, velocities, and positions
    accelerations = forces / masses[:, np.newaxis]
    velocities += accelerations * delta
    positions += velocities * delta

    positions[:, 1] = np.minimum(positions[:, 1], 600)  
    velocities[np.where(positions[:, 1] >= 600), 1] *= -0.5  

# Visualization function
def visualize(surf):
    for s,spring in enumerate(springs):
        i, j = spring
        pg.draw.aaline(surf, (255, 255, 255), positions[i], positions[j],4)

dt = 0.03

def main():
    global dt
    pg.init()
    display = (1280, 720)

    surf = pg.display.set_mode(display)
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