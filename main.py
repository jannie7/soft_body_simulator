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
        angle_step = 2 * np.pi / count
        for i in range(count):
            angle = i * angle_step
            px = x + radius * np.cos(angle)
            py = y + radius * np.sin(angle)
            self.pm_structure.append(point_mass(px, py, m / count))
        
        for i in range(count):
            self.spring_structure.append(Spring(self.pm_structure[i], self.pm_structure[(i + 1) % count], self.k))
            self.spring_structure.append(Spring(self.pm_structure[i], self.pm_structure[(i + 3) % count], self.k))
        
        self.outedge = [self.spring_structure[i * 2] for i in range(count)]
             
class Triangle(Shape):
    def __init__(self, x, y, size, k=20, m=1):
        super().__init__(k)
        height = size * (np.sqrt(3) / 2)
        self.pm_structure = [
            point_mass(x, y - height / 3, m / 3),  # top vertex
            point_mass(x - size / 2, y + height * 2 / 3, m / 3),  # bottom left vertex
            point_mass(x + size / 2, y + height * 2 / 3, m / 3)  # bottom right vertex
        ]
        self.spring_structure = [
            Spring(self.pm_structure[0], self.pm_structure[1], self.k),  # top to bottom left
            Spring(self.pm_structure[1], self.pm_structure[2], self.k),  # bottom left to bottom right
            Spring(self.pm_structure[2], self.pm_structure[0], self.k)  # bottom right to top
        ]
        self.outedge = self.spring_structure

class Camera:
    def __init__(self, pos, scale,display):
        self.pos = pos
        self.scale = scale
        self.aspect_ratio = display[0] / display[1]
        self.Camera_ratio = [self.scale, self.scale/self.aspect_ratio]
        self.display = display
    def ndc_space(self,pos):
        return ((pos[0]- self.pos[0]) / self.Camera_ratio[0], (pos[1]- self.pos[1]) / self.Camera_ratio[1])
    def screen_space(self,pos):
        ndc = self.ndc_space(pos)
        return ((ndc[0]+1) * self.display[0]/2, (1-((ndc[1]+1) /2)) * self.display[1])
    def screen_to_world(self,pos):
        t= ((pos[0] * 2/self.display[0])-1, 1-(pos[1] * 2/self.display[1]))
        return (t[0] * self.Camera_ratio[0] + self.pos[0], t[1] * self.Camera_ratio[1] + self.pos[1])
    


def horizontal_segment_intersection(A, B, C, D):
    x1, y_h = A
    x2, _ = B
    x3, y3 = C
    x4, y4 = D

    #make sure A is to the left of B, if not swap
    if x1 > x2:
        x1, x2 = x2, x1
        
    #check if the x ranges overlap
    if x2 < min( x3,x4) or x1 > max(x4,x3):
        return False
    
    #check if y ranges overlap
    if y_h < min(y3, y4) or y_h > max(y3, y4):
        return False
    
    #slope of line CD
    if x4 == x3:
        return False  # avoid division by zero if line CD is vertical
    m_CD = (y4 - y3) / (x4 - x3)
    #y intercept of line CD
    b_CD = y3 - m_CD * x3

    #x at y_h for line CD
    if m_CD == 0:
        return False  # avoid division by zero if line CD is horizontal
    x_CD = (y_h - b_CD) / m_CD

    #check if x_CD is within the x range of the horizontal segment
    if x_CD < x1 or x_CD > x2:
        return False    
    return True

def check_aabb_collision(bbox1, bbox2):
    x_min1, x_max1, y_min1, y_max1 = bbox1
    x_min2, x_max2, y_min2, y_max2 = bbox2

    if x_max1 < x_min2 or x_max2 < x_min1 or y_max1 < y_min2 or y_max2 < y_min1:
        return False
    return True

def point_to_aabb_check(point, bbox):
    x_min, x_max, y_min, y_max = bbox
    x, y = point

    if x_min <= x <= x_max and y_min <= y <= y_max:
        return True
    return False

def point_inside_shape(point,shape):
    bbox = shape.bbox #need to cache this somewhere
    count = 0
    for spring in shape.outedge:
        if horizontal_segment_intersection( point.pos, [bbox[1]+1, 0],spring.p1.pos, spring.p2.pos):
            count += 1
    return count % 2 == 1

GRAVITY = np.array([0, -9.8],dtype=float)
DELTA = 0.01
DAMPING = 2

def main():
    global dt
    pg.init()
    display = (1280, 720)

    surf = pg.display.set_mode(display)
    pg.display.set_caption("Test")

    world = []
    world.append(Square(0, 10, 2))
    world.append(Square(5, 10, 2))
    world.append(Triangle(10, 10, 2))
    world.append(Circle(-5, 10, 3,10,m=1))



    Main_camera = Camera([5,5], 30,display)
    

    selected_point = None
    mouse_pos = None
    while True:               
        for event in pg.event.get():
            if event.type == pg.QUIT:
                pg.quit()
                quit()  
            if event.type == pg.MOUSEBUTTONDOWN:                
                pos_cursor = np.array(Main_camera.screen_to_world(pg.mouse.get_pos()))
                selected_point = None
                min_dist = float('inf')
                for shape in world:
                    for pm in shape.pm_structure:
                        dist = np.linalg.norm(pm.pos - pos_cursor)
                        if dist < min_dist:
                            min_dist = dist
                            selected_point = pm
                if selected_point and min_dist<Main_camera.scale/10:                    
                    mouse_pos = pos_cursor
                else:
                    selected_point = None
            if event.type == pg.MOUSEBUTTONUP:                
                selected_point = None 

        if selected_point and type(mouse_pos) != type(None):
            mouse_pos = np.array(Main_camera.screen_to_world(pg.mouse.get_pos()))          
            selected_point.pos = mouse_pos
            selected_point.velocity = np.array([0,0],dtype=float)
            selected_point.force = np.array([0,0],dtype=float)
                
        print(Main_camera.screen_to_world(pg.mouse.get_pos()),pg.mouse.get_pos(),selected_point)
        collusion_dict_color = {}
        surf.fill((0, 0, 0, 0))
        for i, shape1 in enumerate(world):
            for shape2 in world[i+1:]:
                if check_aabb_collision(shape1.bbox, shape2.bbox):
                    collusion_dict_color[shape1] = (255, 0, 0)
                    collusion_dict_color[shape2] = (255, 0, 0)

        collusion_info = []

        for shape in world:
            for other_shape in world:
                if shape == other_shape:
                    continue
                B_bbox = other_shape.bbox
                if not check_aabb_collision(shape.bbox, B_bbox):
                    continue
                for point_mass in shape.pm_structure:
                    #skip point if not inside aabb of other shape
                    if(not point_to_aabb_check(point_mass.pos, B_bbox)):
                        continue
                    #check if point is inside the shape
                    if not point_inside_shape(point_mass, other_shape):
                        continue
                    #find closest edge to point mass in other shape and calculate distance and normal
                    min_dist = float('inf')
                    closest_edge = None
                    closest_normal = None
                    interp_value = 0
                    
                    for edge in other_shape.outedge:
                        #calculate perpindicular distance from point to edge
                        dist = np.abs(np.cross(edge.p2.pos - edge.p1.pos, edge.p1.pos - point_mass.pos)) / np.linalg.norm(edge.p2.pos - edge.p1.pos)
                        if dist < min_dist:                           

                            #compute the normal of the edge
                            normal = np.array([edge.p2.pos[1] - edge.p1.pos[1], edge.p1.pos[0] - edge.p2.pos[0]])
                            if not np.linalg.norm(normal) > 0:
                                continue
                            normal = normal / np.linalg.norm(normal)
                            #find point on the edge closest to the point mass
                            t = np.dot(point_mass.pos - edge.p1.pos, edge.p2.pos - edge.p1.pos) / np.dot(edge.p2.pos - edge.p1.pos, edge.p2.pos - edge.p1.pos)
                            #flip the normal if it is pointing away from the point mass
                            if np.dot(normal, point_mass.pos - edge.p1.pos) > 0:
                                normal = -normal
                            min_dist = dist
                            closest_edge = edge
                            closest_normal = normal
                            interp_value = t
                    if closest_edge:
                        collusion_info.append((point_mass, closest_edge, min_dist, closest_normal, interp_value))
                    
        for point_mass, edge, dist, normal, interp_value in collusion_info:
            # Example masses (replace with your actual values)
            m_point = point_mass.mass
            m_p1 = edge.p1.mass
            m_p2 = edge.p2.mass
            total_edge_mass = m_p1 + m_p2

            # Resolve penetration by moving the point mass and edge points
            tolerance = 0.1
            point_mass.pos += normal * (dist + tolerance) * (total_edge_mass / (m_point + total_edge_mass))  # Point mass moves outward

            # Edge points move inward proportionally based on interp_value and masses
            weight_p1 = 1 - interp_value  # Weight for edge.p1
            weight_p2 = interp_value      # Weight for edge.p2

            # edge.p1.pos -= weight_p1 * normal * dist * (m_point / (m_point + total_edge_mass))
            # edge.p2.pos -= weight_p2 * normal * dist * (m_point / (m_point + total_edge_mass))

            # Adjust velocities based on mass ratios
            restitution = 0.8 # Coefficient of restitution
            friction_coefficient = 0.2

            # Normal and tangential components of point mass velocity
            normal_velocity = np.dot(point_mass.velocity, normal) * normal
            tangential_velocity = point_mass.velocity - normal_velocity

            # Adjust point mass velocity (momentum-based scaling)
            point_mass.velocity -= normal_velocity * restitution * (total_edge_mass / (m_point + total_edge_mass))
            point_mass.velocity -= tangential_velocity * friction_coefficient

            # Adjust edge velocities proportionally based on mass and interp_value
            edge.p1.velocity -= weight_p1 * np.dot(edge.p1.velocity, normal) * normal * (m_point / (m_point + total_edge_mass))
            edge.p2.velocity -= weight_p2 * np.dot(edge.p2.velocity, normal) * normal * (m_point / (m_point + total_edge_mass))

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
                if pm.pos[1] < 0:
                    pm.pos[1] = 0
                    pm.velocity[1] *= -1 * 0.5
                    pm.velocity[0] *= 0.1            

            # x_min, x_max, y_min, y_max = shape.bbox
            # pg.draw.rect(surf,collusion_dict_color.get(shape, (0, 255, 0)) , (x_min, y_min, x_max - x_min, y_max - y_min), 1)
        
        for shape in world:
            for spring in shape.spring_structure:
                p1 = Main_camera.screen_space(spring.p1.pos)
                p2 = Main_camera.screen_space(spring.p2.pos)

                pg.draw.aaline(surf, (255, 255, 255), p1, p2, 4)

                


        pg.display.update()


if __name__ == "__main__":
    main()
    