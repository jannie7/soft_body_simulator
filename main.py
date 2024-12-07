import numpy as np
from pygame.locals import *
import pygame as pg
import time


class point_mass:
    def __init__(self, x, y,mass:float=1.0):
        self.mass = mass
        self.pos = np.array([x,y],dtype=float)
        self.velocity = np.array([0.0,0.0],dtype=float)
        self.force = np.array([0.0,0.0],dtype=float)
    def __getitem__(self, index):
        return self.pos[index]
    
class Spring:
    def __init__(self, pm_1 :point_mass, pm_2:point_mass, k=20):
        self.p1 = pm_1
        self.p2 = pm_2
        self.k = k
        self.length = np.linalg.norm(pm_1.pos - pm_2.pos)

class Shape:
    def __init__(self,k=20):
        self.pm_structure = []
        self.spring_structure = []
        self.outedge = []
        self.k = k
    @property
    def bbox(self):
        xs = [pm[0] for pm in self.pm_structure]
        ys = [pm[1] for pm in self.pm_structure]
        x_min = min(xs)
        x_max = max(xs)
        y_min = min(ys)
        y_max = max(ys)
        return x_min, x_max, y_min, y_max
    
class Square(Shape):
    def __init__(self, x, y, size,k=20,m=1):
        super().__init__(k)
        self.pm_structure = [point_mass((x-size), (y+size),m/4), # top left square corner
                                point_mass( (x+size), (y+size),m/4), # top right square corner
                                point_mass( (x+size), (y-size),m/4), # bottom right square corner
                                point_mass( (x-size), (y-size),m/4) # bottom left square corner
                                ]
        self.spring_structure = [Spring(self.pm_structure[0], self.pm_structure[1],self.k), # spring connecting top left, top right
                                 Spring(self.pm_structure[1], self.pm_structure[2],self.k), # spring connecting top right, bottom right
                                 Spring(self.pm_structure[2],self. pm_structure[3],self.k), # spring connecting bottom right, bottom left
                                 Spring(self.pm_structure[3], self.pm_structure[0],self.k), # spring connecting bottom left, top left                                 
                                 Spring(self.pm_structure[0], self.pm_structure[2],self.k), # spring connecting top left, bottom right
                                 Spring(self.pm_structure[1], self.pm_structure[3],self.k)] # spring connecting top right, bottom left
        self.outedge = [self.spring_structure[0],self.spring_structure[1],self.spring_structure[2],self.spring_structure[3]]

class Circle(Shape):
    def __init__(self, x, y, radius,count=20,k=20,m=1):
        super().__init__(k)
        pmass= 1/ count+1
        self.pm_structure = [point_mass(x, y,pmass)]
        self.spring_structure = []
        dangle = 2*np.pi / count
        for i in range(1, count+1):
            angle = i * dangle
            self.pm_structure.append(point_mass(x + radius * np.cos(angle), y + radius * np.sin(angle),pmass))
            self.spring_structure.append(Spring(self.pm_structure[i - 1], self.pm_structure[i],self.k))
            self.outedge.append(self.spring_structure[-1])
            self.spring_structure.append(Spring(self.pm_structure[0], self.pm_structure[i],self.k))
        self.outedge.pop(0)
        self.spring_structure.append(Spring(self.pm_structure[1], self.pm_structure[-1],self.k))
        self.outedge.append(self.spring_structure[-1])     

class Triangle(Circle):
    def __init__(self, x, y, size, k=20,m=1):
        size =2* size / (np.sqrt(3))
        super().__init__(x, y, size,3,k,m)

def horizontal_segment_intersection(A, B, C, D):
    x1, y_h = A
    x2, _ = B
    x3, y3 = C
    x4, y4 = D

    if x1 > x2:
        x1, x2 = x2, x1

    if y3 == y4:
        if y_h != y3:
            return False, None
        
        overlap_start = max(x1, min(x3, x4))
        overlap_end = min(x2, max(x3, x4))
        if overlap_start <= overlap_end:
            return True, (overlap_start, y_h)  
        return False, None

    if y4 - y3 == 0:
        return False, None 
    
    u = (y_h - y3) / (y4 - y3)

    if u < 0 or u > 1:
        return False, None

    x = x3 + u * (x4 - x3)

    if x1 <= x <= x2:
        return True, (x, y_h)

    return False, None

def check_aabb_collision(bbox1, bbox2):
    x_min1, x_max1, y_min1, y_max1 = bbox1
    x_min2, x_max2, y_min2, y_max2 = bbox2

    if x_max1 < x_min2 or x_max2 < x_min1 or y_max1 < y_min2 or y_max2 < y_min1:
        return False
    return True

GRAVITY = np.array([0, 9.81*5],dtype=float)
DELTA = 0.01
DAMPING = 2

def main():
    global dt
    pg.init()
    display = (1280, 720)

    surf = pg.display.set_mode(display)
    pg.display.set_caption("Test")

    world = []
    world.append(Square(100, 100, 50))
    world.append(Triangle(200, 200, 50))
    world.append(Circle(300, 300, 60,9,100))
    world.append(Circle(700, 300, 60,4,100))
    selected_point = None
    mouse_pos = None
    while True:               
        for event in pg.event.get():
            if event.type == pg.QUIT:
                pg.quit()
                quit()  
            if event.type == pg.MOUSEBUTTONDOWN:                
                x, y = pg.mouse.get_pos()
                selected_point = None
                min_dist = float('inf')
                for shape in world:
                    for pm in shape.pm_structure:
                        dist = np.linalg.norm(pm.pos - np.array([x, y]))
                        if dist < min_dist:
                            min_dist = dist
                            selected_point = pm
                if selected_point and min_dist<50:                    
                    mouse_pos = np.array([x, y],dtype=float)
                else:
                    selected_point = None
            if event.type == pg.MOUSEBUTTONUP:                
                selected_point = None 

        if selected_point and type(mouse_pos) != type(None):
            x, y = pg.mouse.get_pos()
            mouse_pos = np.array([x, y],dtype=float)            
            selected_point.pos = mouse_pos
            selected_point.velocity = np.array([0,0],dtype=float)
            selected_point.force = np.array([0,0],dtype=float)
                

        surf.fill((0, 0, 0, 0))
        for shape in world:    
            for spring in shape.spring_structure:                         
                dis = spring.p1.pos - spring.p2.pos   
                 
                new_length = np.linalg.norm(dis)
                dir =   dis / new_length
                spring.force = -spring.k * (new_length - spring.length) * dir
                damping_force = -DAMPING * np.sum((spring.p1.velocity - spring.p2.velocity) * dir) * dir
                spring.p1.force += spring.force + damping_force
                spring.p2.force -= spring.force + damping_force
            for pm in shape.pm_structure:
                pm.force += GRAVITY * pm.mass
                pm.velocity += pm.force / pm.mass * DELTA
                pm.pos += pm.velocity * DELTA
                pm.force *= 0.0
                if pm.pos[1] > 600:
                    pm.pos[1] = 600
                    pm.velocity[1] *= -1 * 0.5
                    pm.velocity[0] *= 0.1

            
            for spring in shape.spring_structure:
                i, j = spring.p1.pos, spring.p2.pos
                pg.draw.aaline(surf, (255, 255, 255), i, j,4)

           
            x_min, x_max, y_min, y_max = shape.bbox
            pg.draw.rect(surf, (0, 255, 0), (x_min, y_min, x_max - x_min, y_max - y_min), 1)
        
        
        
    
        pg.display.update()


if __name__ == "__main__":
    main()
    