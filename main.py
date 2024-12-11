import numpy as np
from pygame.locals import *
import pygame as pg
import time

import pygame_gui as gui
import math


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
        self.inner_springs = []
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
    def make_innerSprings(self):
        #inner springs are springs that are not outedge
        self.inner_springs = [spring for spring in self.spring_structure if spring not in self.outedge]
    
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
        self.make_innerSprings()

class Circle(Shape):
    def __init__(self, x, y, radius,count=20,k=20,m=1):
        super().__init__(k)        
        angle_step = 2 * np.pi / count
        for i in range(count):
            angle = i * angle_step
            px = x + radius * np.cos(angle)
            py = y + radius * np.sin(angle)
            self.pm_structure.append(point_mass(px, py, m / count))
        
        for i in range(count):
            self.spring_structure.append(Spring(self.pm_structure[i], self.pm_structure[(i + 1) % count], self.k))
            self.spring_structure.append(Spring(self.pm_structure[i], self.pm_structure[(i + 2) % count], self.k))
            self.spring_structure.append(Spring(self.pm_structure[i], self.pm_structure[(i + 3) % count], self.k))
        
        self.outedge = [self.spring_structure[i*3] for i in range(count)]

        for i in range(count):
            spring = Spring(self.pm_structure[i], self.pm_structure[(i + count // 2) % count], self.k)
            if not any((spring.p1 == s.p1 and spring.p2 == s.p2) or (spring.p1 == s.p2 and spring.p2 == s.p1) for s in self.spring_structure):
                self.spring_structure.append(spring)
        self.make_innerSprings()
             
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
        self.make_innerSprings()


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
    
    def ndc_space_array(self, positions):
        return (positions - self.pos) / self.Camera_ratio

    def screen_space_array(self, positions):
        ndc = self.ndc_space_array(positions)
        ndc[:, 1] *= -1
        return ((ndc + 1) * np.array([self.display[0] / 2, self.display[1] / 2]))
    
    def update_scale(self, scale):
        self.scale = scale
        self.Camera_ratio = [self.scale, self.scale/self.aspect_ratio]

def horizontal_segment_intersection(A, B, C, D):
    x1, y_h = A
    x2, _ = B
    x3, y3 = C
    x4, y4 = D

    # Ensure A is to the left of B
    if x1 > x2:
        x1, x2 = x2, x1

    # Check if y_h is in range of y3 and y4
    if y_h < min(y3, y4) or y_h > max(y3, y4):
        return False
    
    if y3 == y4:  # Horizontal line
        return False


    # Calculate intersection point of line CD with horizontal line y = y_h
    if x3 == x4:  # Vertical line
        if x3 < x1 or x3 > x2:
            return False
        x_intersection = x3
    else:
        # Line equation y = mx + b
        m = (y4 - y3) / (x4 - x3)
        b = y3 - m * x3
        x_intersection = (y_h - b) / m

    # Check if intersection is within segment bounds
    if x1 <= x_intersection <= x2:
        return True
    return False

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

def resolve_collision(dist, point_mass, interp_value, edge, normal):
    m_point = point_mass.mass
    m_p1 = edge.p1.mass
    m_p2 = edge.p2.mass
    total_edge_mass = m_p1 + m_p2

    # Resolve penetration by moving the point mass and edge points
    tolerance = 0.001
    point_mass.pos += normal * (dist + tolerance) * (total_edge_mass / (m_point + total_edge_mass))  # Point mass moves outward

    # Edge points move inward proportionally based on interp_value and masses
    weight_p1 = 1 - interp_value  # Weight for edge.p1
    weight_p2 = interp_value      # Weight for edge.p2

    edge.p1.pos -= weight_p1 * normal * dist * (m_point / (m_point + total_edge_mass))
    edge.p2.pos -= weight_p2 * normal * dist * (m_point / (m_point + total_edge_mass))

    # Adjust velocities based on mass ratios
    restitution = 0.99 # Coefficient of restitution
    friction_coefficient = 0.9  # Coefficient of friction

    # Normal and tangential components of point mass velocity
    normal_velocity = np.dot(point_mass.velocity, normal) * normal
    #get tangent vector
    tangent = np.array([-normal[1], normal[0]])
    tangential_velocity = np.dot(point_mass.velocity, tangent) * tangent

    # Adjust point mass velocity (momentum-based scaling)
    point_mass.velocity -= normal_velocity * (restitution) * (total_edge_mass / (m_point + total_edge_mass))
    point_mass.velocity -= tangential_velocity * friction_coefficient

    # Adjust edge velocities proportionally based on mass and interp_value
    edge.p1.velocity -= weight_p1 * np.dot(edge.p1.velocity, normal) * normal * (m_point / (m_point + total_edge_mass))
    edge.p2.velocity -= weight_p2 * np.dot(edge.p2.velocity, normal) * normal * (m_point / (m_point + total_edge_mass))

    # #draw normal and tangent
    # pg.draw.aaline(surf, (255, 0, 0), Main_camera.screen_space(point_mass.pos), Main_camera.screen_space(point_mass.pos + normal * 10), 4)
    # pg.draw.aaline(surf, (0, 0, 255), Main_camera.screen_space(point_mass.pos), Main_camera.screen_space(point_mass.pos + tangent * 10), 4)

def resolve_collision2(dist, point_mass, interp_value, edge, normal):
    tolerance = TOLORANCE

    if(dist <= tolerance):
        return
    
    
    # Calculate the virtual point for the edge
    total_mass = edge.p1.mass + edge.p2.mass
    m_point = point_mass.mass
    # Distribute virtual point force back to edge points
    weight_p1 = edge.p1.mass / total_mass
    weight_p2 = edge.p2.mass / total_mass

    
    # Penetration resolution for point_mass and virtual_point
    

    pmove = normal * (dist + tolerance) * (total_mass / (point_mass.mass + total_mass))
    emove = normal * (dist + tolerance) * (point_mass.mass / (point_mass.mass + total_mass))

    point_mass.pos += pmove

    edge.p1.pos -= emove * (1-interp_value) * weight_p1
    edge.p2.pos -= emove * (interp_value) * weight_p2
    
    # #set all velocities to zero
    # point_mass.velocity *= 0
    # edge.p1.velocity *= 0
    # edge.p2.velocity *= 0

    virtual_point_pos = (edge.p1.mass * edge.p1.pos + edge.p2.mass * edge.p2.pos) / total_mass
    virtual_point_velocity = (edge.p1.mass * edge.p1.velocity + edge.p2.mass * edge.p2.velocity) / total_mass
    
    # Adjust virtual point position
    # virtual_point_pos -= normal * dist * (m_point / (m_point + total_mass))

    # Calculate velocity adjustments
    restitution =0.9  # Coefficient of restitution
    friction_coefficient = 0.5  # Coefficient of friction

    # Normal and tangential velocity components
    relative_velocity = point_mass.velocity - virtual_point_velocity
    normal_velocity = np.dot(relative_velocity, normal)
    if normal_velocity > 0.0001:
        return
    normal_velocity = normal_velocity * normal
    tangent = np.array([-normal[1], normal[0]])
    tangential_velocity = np.dot(relative_velocity, tangent) * tangent
    #tangential_velocity = relative_velocity - normal_velocity

    impulse = normal_velocity * restitution * point_mass.mass * total_mass / (point_mass.mass + total_mass)
    point_mass.velocity -= impulse / point_mass.mass
    edge.p1.velocity += impulse*(1-interp_value)*weight_p1 / edge.p1.mass
    edge.p2.velocity += impulse*(interp_value)*weight_p2  / edge.p2.mass

    friction_impulse = tangential_velocity * friction_coefficient * point_mass.mass
    point_mass.velocity -= friction_impulse / point_mass.mass
    edge.p1.velocity += friction_impulse*(1-interp_value)*weight_p1  / edge.p1.mass
    edge.p2.velocity += friction_impulse *(interp_value)*weight_p2 / edge.p2.mass


    


    #


GRAVITY = np.array([0, -9.8],dtype=float)
DELTA = 0.016
DAMPING = 0.5
TOLORANCE = 0.0001

def main():
    pg.init()
    display = (1280, 720)
    surf = pg.display.set_mode(display)
    pg.display.set_caption("Test")

    manager = gui.UIManager(display)

    Mode_btn = gui.elements.UIButton(relative_rect=pg.Rect((1280-150-10, 10), (150, 25)),
                                             text='Switch Mode',
                                             manager=manager)
    Render_Springs_btn = gui.elements.UIButton(relative_rect=pg.Rect((1280-150-10, 40), (150, 25)),
                                             text='Toggle Springs',
                                             manager=manager)
    Render_transfrom_btn = gui.elements.UIButton(relative_rect=pg.Rect((1280-150-10, 70), (150, 25)),
                                             text='Transform Mode',
                                             manager=manager)
    
    details_test = gui.elements.UILabel(relative_rect=pg.Rect((10, 10), (-1, -1)),
                                             text='SoftBody Simulation',
                                             manager=manager)
    details_Mode = gui.elements.UILabel(relative_rect=pg.Rect((10, 30+10), (-1, -1)),
                                             text='Mode: Linear Algebra',
                                             manager=manager)
    details_fps = gui.elements.UILabel(relative_rect=pg.Rect((10, 50+10), (-1, -1)),
                                             text='FPS/dt: 0/0ms',
                                             manager=manager)
    details_sim = gui.elements.UILabel(relative_rect=pg.Rect((10, 70+10), (-1, -1)),
                                             text='Sim time: 0ms',
                                             manager=manager)
    details_col = gui.elements.UILabel(relative_rect=pg.Rect((10, 90+10), (-1, -1)),
                                             text='Col time: 0ms',
                                             manager=manager)
    details_viz = gui.elements.UILabel(relative_rect=pg.Rect((10, 110+10), (-1, -1)),
                                             text='Viz time: 0ms',
                                             manager=manager)
    details_other = gui.elements.UILabel(relative_rect=pg.Rect((10, 130+10), (-1, -1)),
                                             text='Pcount: 0',
                                             manager=manager)
    
    
    Main_camera = Camera(np.array([5.0,5.0]), 15,display)

    world = []

    # world.append(Square(0, 10, 0.5,5,m=0.1))
    # world.append(Square(5, 10, 2.5))
    # world.append(Triangle(10, 10, 2.5,m=0.5))
    # world.append(Circle(-1, 20, 1,10,m=1,k=100))
    # world.append(Circle(4, 20, 0.5,11,m=1,k=100))


    num_shapes = 10
    rows = math.ceil(math.sqrt(num_shapes))
    cols = math.ceil(num_shapes / rows)
    spacing = 3  # Adjust spacing as needed

    for i in range(rows):
        for j in range(cols):
            if len(world) >= num_shapes:
                break
            x = j * spacing
            y = i * spacing
            if (i + j) % 2 == 0:
                world.append(Square(x, y, size=1, k=40, m=1))
            else:
                world.append(Circle(x, y, radius=1, count=11, k=20, m=1))
            

    #numpy representation of world
    points_ref = [pm for shape in world for pm in shape.pm_structure]
    springs = np.array([np.array([points_ref.index(spring.p1),points_ref.index(spring.p2)]) for shape in world for spring in shape.spring_structure ])
    outedge = np.array([np.array([points_ref.index(spring.p1),points_ref.index(spring.p2)]) for shape in world for spring in shape.outedge ])
    inner_springs = np.array([np.array([points_ref.index(spring.p1),points_ref.index(spring.p2)]) for shape in world for spring in shape.inner_springs ])    
    lengths = np.array([spring.length for shape in world for spring in shape.spring_structure])
    masses = np.array([pm.mass for shape in world for pm in shape.pm_structure])
    spring_constants = np.array([spring.k for shape in world for spring in shape.spring_structure])

    positions = np.array([pm.pos for pm in points_ref], dtype=float)
    velocities = np.array([pm.velocity for pm in points_ref], dtype=float)
    forces = np.array([pm.force for pm in points_ref], dtype=float)

    for i, pm in enumerate(points_ref):
        pm.pos = positions[i]
        pm.velocity = velocities[i]
        pm.force = forces[i]

    

    print(f"shape of springs {springs.shape}")
    print(f"shape of positions {positions.shape}")
    print(f"shape of lengths {lengths.shape}")
    print(f"shape of masses {masses.shape}")
    print(f"shape of velocities {velocities.shape}")
    print(f"shape of forces {forces.shape}")
    print(f"shape of spring_constants {spring_constants.shape}")


    
    
    selected_point = None
    mouse_pos = None
    pan_mode = False
    pan_start = None
    dt = DELTA

    numpy_mode = True
    show_spring = False
    render_mode = False

    simulation_time = 0
    visualization_time = 0
    shape_viz_time = 0
    collision_time = 0
    dt_sum = 0.001

    last_ui_update = 0

    count = 0


    while True:   
        count += 1
        start = time.perf_counter()
        #event and input handling            
        for event in pg.event.get():
            mode_changed = False
            if event.type == pg.QUIT:
                pg.quit()
                quit()  
            if event.type == pg.MOUSEBUTTONDOWN:   
                if(event.button == 1):
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
                elif(event.button == 2):
                    pan_mode = True
                    pan_start = np.array(pg.mouse.get_pos())
            if event.type == pg.MOUSEBUTTONUP:                
                selected_point = None 
                pan_mode = False
            if event.type == pg.MOUSEWHEEL:
                direction = event.y>0
                target_scale = np.clip(Main_camera.scale * (1.1 if direction else 0.9),0.1,1000)
                Main_camera.update_scale(target_scale)
            if event.type == gui.UI_BUTTON_PRESSED:
                if event.ui_element == Mode_btn:   
                    numpy_mode = not numpy_mode 
                    if(numpy_mode):
                            #sync
                            start_conv = time.perf_counter()
                            points_ref = [pm for shape in world for pm in shape.pm_structure]
                            springs = np.array([np.array([points_ref.index(spring.p1),points_ref.index(spring.p2)]) for shape in world for spring in shape.spring_structure ])
                            outedge = np.array([np.array([points_ref.index(spring.p1),points_ref.index(spring.p2)]) for shape in world for spring in shape.outedge ])
                            inner_springs = np.array([np.array([points_ref.index(spring.p1),points_ref.index(spring.p2)]) for shape in world for spring in shape.inner_springs ])
                            positions = np.array([pm.pos for pm in points_ref])
                            lengths = np.array([spring.length for shape in world for spring in shape.spring_structure])
                            masses = np.array([pm.mass for shape in world for pm in shape.pm_structure])
                            velocities = np.array([pm.velocity for shape in world for pm in shape.pm_structure])
                            forces = np.array([pm.force for shape in world for pm in shape.pm_structure])
                            spring_constants = np.array([spring.k for shape in world for spring in shape.spring_structure]) 
                            
                            for i, pm in enumerate(points_ref):
                                pm.pos = positions[i]
                                pm.velocity = velocities[i]
                                pm.force = forces[i]
                            #print(f"conversion time {time.perf_counter()-start_conv}")
                    if(not numpy_mode):
                        start_conv = time.perf_counter()
                        for i,point in enumerate(points_ref):
                            point.pos = positions[i]
                            point.velocity = velocities[i]
                            point.force = forces[i]     
                        #print(f"conversion time {time.perf_counter()-start_conv}")
                    
                    mode_changed = True
                if(event.ui_element == Render_Springs_btn):
                    show_spring = not show_spring
                    mode_changed = True
                if(event.ui_element == Render_transfrom_btn):
                    render_mode = not render_mode
                    mode_changed = True
            manager.process_events(event)
            if mode_changed:
                visualization_time = 0
                simulation_time = 0
                collision_time = 0
                shape_viz_time = 0
                count = 1
                dt_sum = 0.001
        if selected_point and type(mouse_pos) != type(None):
            mouse_pos = np.array(Main_camera.screen_to_world(pg.mouse.get_pos()))          
            selected_point.pos += (-selected_point.pos + mouse_pos)
            selected_point.velocity *= 0
            selected_point.force **= 0
        if pan_mode:
            pan_end = np.array(pg.mouse.get_pos())
            pan_amt =(pan_end - pan_start)/Main_camera.display*2 * Main_camera.Camera_ratio
            Main_camera.pos[0] -= pan_amt[0]
            Main_camera.pos[1] += pan_amt[1] 
            pan_start = pan_end
        manager.update(dt)
        
        collusion_info = []

        start_sim = time.perf_counter()
        delta_sim = min(0.016,dt)
        #simulation
        if not numpy_mode:
            for shape in world:    
                for spring in shape.spring_structure:                         
                    dis = spring.p1.pos - spring.p2.pos                  
                    new_length = np.linalg.norm(dis)
                    dir =   dis / new_length
                    spring.force = -spring.k * (new_length - spring.length) * dir
                    damping_force = -DAMPING * np.dot((spring.p1.velocity - spring.p2.velocity) , dir) * dir
                    spring.p1.force += spring.force + damping_force
                    spring.p2.force -= spring.force + damping_force
                for pm in shape.pm_structure:
                    pm.force += GRAVITY * pm.mass
                    pm.velocity += pm.force / pm.mass * delta_sim
                    pm.pos += pm.velocity * delta_sim
                    pm.force *= 0.0
                    if pm.pos[1] < 0:
                        pm.pos[1] = 0
                        pm.velocity[1] *= -0.5
                        pm.velocity[0] *= 0.1   
        else:
            forces.fill(0)
            forces += GRAVITY * masses[:, np.newaxis]     
            displacement = positions[springs[:, 1]] - positions[springs[:, 0]]  # shape: (num_springs, 2)
    
            lengths_current = np.linalg.norm(displacement, axis=1)  # shape: (num_springs,)

            direction = displacement / lengths_current[:, None]  # shape: (num_springs, 2)

            spring_force = (spring_constants * (lengths_current - lengths))[:, None] * direction  # shape: (num_springs, 2)

            relative_velocity = velocities[springs[:, 1]] - velocities[springs[:, 0]]  # shape: (num_springs, 2)

            damping_force = DAMPING * np.sum(relative_velocity * direction, axis=1)[:, None]  * direction  # shape: (num_springs, 2)

            total_force = spring_force + damping_force  # shape: (num_springs, 2)
        
            np.add.at(forces, springs[:, 0], total_force)  
            np.add.at(forces, springs[:, 1], -total_force)

            # Update accelerations, velocities, and positions
            accelerations = forces / masses[:, np.newaxis]
            velocities += accelerations * delta_sim
            positions += velocities * delta_sim

            positions[:, 1] = np.maximum(positions[:, 1], 0)  
            velocities[positions[:, 1] <= 0] *= np.array([0.1, -0.5])
        simulation_time += time.perf_counter()-start_sim

        start_collission = time.perf_counter()
        #collusion detection
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
                    hit_point = None
                    interp_value = 0
                    for edge in other_shape.outedge:
                        #calculate perpindicular distance from point to edge
                        dist = np.cross(edge.p2.pos - edge.p1.pos, edge.p1.pos - point_mass.pos)
                        dist = np.abs(dist) / np.linalg.norm(edge.p2.pos - edge.p1.pos)
                        if dist < min_dist:
                            #compute the normal of the edge
                            normal = np.array([edge.p2.pos[1] - edge.p1.pos[1], edge.p1.pos[0] - edge.p2.pos[0]])
                            if not np.linalg.norm(normal) > 0.00001:
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
                    
                    # for edge in other_shape.outedge:
                    #     p1 = edge.p1.pos
                    #     p2 = edge.p2.pos

                    #     toP = point_mass.pos-p1                      
                    #     edge_ = p2 - p1
                    #     normal = np.array([-edge_[1], edge_[0]])
                    #     dist_ = np.linalg.norm(edge_)
                    #     normal /= dist_
                    #     edge_dir = edge_ / dist_

                    #     dist= float('inf')

                    #     x = np.dot(toP, edge_dir)
            
                    #     if x <= 0:
                    #         dist = np.linalg.norm(toP)
                    #         hit_point = p1
                    #         interp_value = 0
                    #     elif x >= dist_:
                    #         dist = np.linalg.norm(point_mass.pos - p2)
                    #         hit_point = p2
                    #         interp_value = 1
                    #     else:
                    #         #perpendicular distance from point to edge
                    #         dist = np.abs(np.cross(edge_, toP)) / dist_                        
                    #         hit_point = p1 + (x * edge_dir)
                    #         interp_value = x / dist_

                    #     if dist < min_dist:
                    #         min_dist = dist
                    #         closest_edge = edge
                    #         closest_normal = normal
                    #         #check if normal points away from point mass
                    #         if np.dot(normal, point_mass.pos - hit_point) > 0:
                    #             normal = -normal
                    if closest_edge:
                        collusion_info.append((point_mass, closest_edge, min_dist, closest_normal, interp_value))
                                         
        #collusion resolution           
        for point_mass, edge, dist, normal, interp_value in collusion_info:
            #Example masses (replace with your actual values)
            resolve_collision2(dist, point_mass, interp_value, edge, normal)
            
            # tangent = np.array([-normal[1], normal[0]])
            # pg.draw.aaline(surf, (255, 0, 0), Main_camera.screen_space(point_mass.pos), Main_camera.screen_space(point_mass.pos + normal * 1), 4)
            # pg.draw.aaline(surf, (0, 0, 255), Main_camera.screen_space(point_mass.pos), Main_camera.screen_space(point_mass.pos + tangent * 1), 4)
        collision_time += time.perf_counter()-start_collission

        start_viz = time.perf_counter()
        surf.fill((0, 0, 0, 0))
        #drawing
        if render_mode:#not numpy_mode:
            for shape in world:
                for spring in shape.outedge:
                    p1 = Main_camera.screen_space(spring.p1.pos)
                    p2 = Main_camera.screen_space(spring.p2.pos)
                    if p1[0] < 0 or p1[0] > display[0] or p1[1] < 0 or p1[1] > display[1]:
                        continue
                    pg.draw.aaline(surf, (255, 255, 255), p1, p2)
                if show_spring:
                    for spring in shape.inner_springs:
                        p1 = Main_camera.screen_space(spring.p1.pos)
                        p2 = Main_camera.screen_space(spring.p2.pos)
                        if p1[0] < 0 or p1[0] > display[0] or p1[1] < 0 or p1[1] > display[1]:
                            continue
                        pg.draw.aaline(surf, (0, 255, 0), p1, p2)
        else:
            screen_space_positions = Main_camera.screen_space_array(positions)
            
            for spring in outedge:
                p1 = screen_space_positions[spring[0]]
                p2 = screen_space_positions[spring[1]]
                if p1[0] < 0 or p1[0] > display[0] or p1[1] < 0 or p1[1] > display[1]:
                    continue
                pg.draw.aaline(surf, (255, 255, 255), p1, p2)
            if show_spring:
                for spring in inner_springs:
                    p1 = screen_space_positions[spring[0]]
                    p2 = screen_space_positions[spring[1]]
                    if p1[0] < 0 or p1[0] > display[0] or p1[1] < 0 or p1[1] > display[1]:
                        continue
                    pg.draw.aaline(surf, (0, 255, 255), p1, p2)
        
        shape_viz_time += time.perf_counter()-start_viz

        if(time.perf_counter()-last_ui_update > 0.250):
            last_ui_update = time.perf_counter()
            details_Mode.set_text(f"Mode: {'Numpy (Linear Algebra)' if numpy_mode else 'Sequential'}")
            details_fps.set_text(f"FPS/dt (Avg): {1/(dt_sum/count):.0f}/{dt_sum*1000/count:.2f}ms (Current: {dt*1000:0.1f}ms)")
            details_sim.set_text(f"Sim time: {simulation_time*1000/count:.3f}ms")
            details_col.set_text(f"Col time: {collision_time*1000/count:.3f}ms")
            details_viz.set_text(f"Viz time: {visualization_time*1000/count:.3f}ms (Shape time: {shape_viz_time*1000/count:.3f}ms)")
            details_other.set_text(f"Point count: {len(points_ref)} Spring count: {len(springs)} Shape count: {len(world)} Render mode: {'SEQ' if render_mode else 'LA'}")

        if(count>5000):
            dt_sum = 500 *dt_sum/count
            simulation_time = 500 *simulation_time/count
            visualization_time = 500 *visualization_time/count
            collision_time = 500 *collision_time/count
            shape_viz_time = 500 *shape_viz_time/count
            count = 500

        
        manager.draw_ui(surf)
        pg.display.update()
        visualization_time += time.perf_counter()-start_viz
        
        dt = time.perf_counter()-start
        dt_sum += dt






if __name__ == "__main__":
    main()
    