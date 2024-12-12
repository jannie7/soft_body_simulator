#include <iostream>
#include <stdio.h>
#include <cmath>
#include <vector>
#include <math.h> // Access functions for sin, cos, tan; use radians
#include <thread>
#include <omp.h> // For openMP 
#include <chrono>
#include <array>
#include <SDL.h>
#include "imgui.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_sdlrenderer2.h"

#define PI 3.14159265358979323846
#define WINDOW_WIDTH 1280   // Set the desired window width
#define WINDOW_HEIGHT 720
using namespace std;
using namespace std::chrono;

// Constants

const float DELTA = 0.01f;
const float DAMPING = 0.5f;

class Vector2D { // define a vector object with x and y components to be utilized by other classes; 2-d physics simulation, so every vector (position, velocity, force) will be 2-d.
public:
    float x, y;
    Vector2D(float x = 0.0f, float y = 0.0f)
        : x(x), y(y) {} // constructor

    // Define vector operations 

    // Vector subtraction
    Vector2D operator-(const Vector2D& sub_vector) const {
        return Vector2D(x - sub_vector.x, y - sub_vector.y);
    }

    // Vector addition (return new vector object)
    Vector2D operator+(const Vector2D& add_vector) const {
        return Vector2D(x + add_vector.x, y + add_vector.y);
    }

    // Scalar x vector multiplication
    Vector2D operator*(float scalar) const {
        return Vector2D(x * scalar, y * scalar);
    }

    // Vector / scalar division
    Vector2D operator/(float scalar) const {
        return Vector2D(x / scalar, y / scalar);
    }

    // Scalar multiplication (friend function)
    friend Vector2D operator*(float scalar, const Vector2D& vector) {
        return Vector2D(vector.x * scalar, vector.y * scalar);
    }

    // In-place Vector addition (return original vector object)
    Vector2D& operator+=(const Vector2D& add_vector) {
        x += add_vector.x;
        y += add_vector.y;
        return *this; // Returns this instance of Vector2d, now updated in-place
    }

    // In-place Vector subtraction (return original vector object)
    Vector2D& operator-=(const Vector2D& sub_vector) {
        x -= sub_vector.x;
        y -= sub_vector.y;
        return *this; // Returns this instance of Vector2D
    }

    // Access vector length easily
    float length() const {
        return sqrt(x * x + y * y);
    }

    // Normalization of a vector (a unit vector is the vector divided by its magnitude - mag of 1 with saved direction)
    void normalize() {
        float len = length();
        if (len > 0) {
            x /= len;
            y /= len;
        }
    }

    // Dot product (vector multiplication)
    float dot(const Vector2D& vector2) const {
        return x * vector2.x + y * vector2.y;
    }

    // Cross product
    float cross(const Vector2D& vector2) const {
        return x * vector2.y - y * vector2.x;
    }
};

const Vector2D GRAVITY = Vector2D(0.0f, -9.81f);

class PointMass { //Define a point mass
public:
    Vector2D pos; // Every point mass will be defined by its own position, velocity, and force.
    Vector2D velocity;
    Vector2D force;
    float mass;
    SDL_Color color;
    bool isBeingDragged;
    SDL_Point position;

    PointMass(float x, float y, float mass = 1.0f)
        : pos(x, y), velocity(0, 0), force(0, 0), mass(mass), color({ 255, 0, 0, 255 }), isBeingDragged(false) { // when a point mass is initialized, it will be at rest
        position.x = static_cast<int>(x);                                                                       // at rest => force = 0, velocity = 0
        position.y = static_cast<int>(y);
    }

    void update_position_from_vector() {
        position.x = static_cast<int>(pos.x);
        position.y = static_cast<int>(pos.y);
    }

    Vector2D get_position_item(int index) {
        if (index == 0) {
            return pos.x;
        }
        else {
            return pos.y;
        }
    }
};

class Spring { // Define a spring, which connects point masses 
public:
    PointMass* p1; // reference an existing PointMass instance
    PointMass* p2;
    float k;
    float length;

    Spring(PointMass* p1, PointMass* p2, float k) : p1(p1), p2(p2), k(k) {
        length = (p1->pos - p2->pos).length();
    }
};

class Shape {
public:
    vector<PointMass*> pm_structure;
    vector<Spring> spring_structure;
    vector<Spring> outedge;
    float k;

    std::array<float, 4> bbox() {
        float x_min = pm_structure[0]->pos.x;
        float x_max = pm_structure[0]->pos.x;
        float y_min = pm_structure[0]->pos.y;
        float y_max = pm_structure[0]->pos.y;

        for (const auto& pm : pm_structure) {
            x_min = std::min(x_min, pm->pos.x);
            x_max = std::max(x_max, pm->pos.x);
            y_min = std::min(y_min, pm->pos.y);
            y_max = std::max(y_max, pm->pos.y);
        }

        return { x_min, x_max, y_min, y_max }; // Return as std::array
    }

    void update_forces(float delta) {

            for (auto& spring : spring_structure) {                         
                Vector2D dis = spring.p1->pos - spring.p2->pos;                  
                float new_length = dis.length(); // Equivalent to np.linalg.norm(dis)
                Vector2D dir = dis / new_length;
                auto spring_force = -spring.k*2 * (new_length - spring.length) * dir;
                Vector2D damping_force = -DAMPING * (spring.p1->velocity - spring.p2->velocity).dot(dir) * dir;
                spring.p1->force += spring_force + damping_force;
                spring.p2->force -= spring_force + damping_force;
            }
            for (auto& pm : pm_structure) {
                pm->force += GRAVITY * pm->mass;
                pm->velocity += (pm->force / pm->mass) * delta;
                pm->pos += pm->velocity * delta;
                pm->force = Vector2D(0, 0);
                if (pm->pos.y < 0) {
                    pm->pos.y = 0;
                    pm->velocity.y *= -0.5;
                    pm->velocity.x *= 0.1;  
                }
            }
        
        }
    
};

class Square : public Shape {
public:
    Square(float x, float y, float size, float k = 20.0f, float m = 1.0f) {
        m = m / 4.0f;
        /* Define the starting structure of a square, made of point masses.
        Dependent on a starting position (x, y) */
        pm_structure.push_back(new PointMass(x - size, y + size, m)); // top left; storing by reference since Spring will reference 
        pm_structure.push_back(new PointMass(x + size, y + size, m)); // top right
        pm_structure.push_back(new PointMass(x + size, y - size, m)); // bottom right
        pm_structure.push_back(new PointMass(x - size, y - size, m)); // bottom left

        spring_structure.push_back(Spring(pm_structure[0], pm_structure[1], k)); // top left to top right; storing by value for efficient access
        spring_structure.push_back(Spring(pm_structure[1], pm_structure[2], k)); // top right to bottom right
        spring_structure.push_back(Spring(pm_structure[2], pm_structure[3], k)); // bottom right to bottom left
        spring_structure.push_back(Spring(pm_structure[3], pm_structure[0], k)); // bottom left to top left
        spring_structure.push_back(Spring(pm_structure[0], pm_structure[2], k)); // diagonal
        spring_structure.push_back(Spring(pm_structure[1], pm_structure[3], k)); // diagonal

        for (int edge = 0; edge < 4; edge++) {
            outedge.push_back(spring_structure[edge]);
        }
    }
};

class Circle : public Shape {
public:
    Circle(float x, float y, float radius, int count = 20, float k = 20.0f, float m = 1.0f)  {
        float angle_step = 2 * PI / count;
        for (int i = 0; i < count; ++i) {
            float angle = i * angle_step;
            float px = x + radius * cos(angle);
            float py = y + radius * sin(angle);
            pm_structure.push_back(new PointMass(px, py, m / count));
        }

        for (int i = 0; i < count; ++i) {
            spring_structure.push_back(Spring(pm_structure[i], pm_structure[(i + 1) % count], k));
            spring_structure.push_back(Spring(pm_structure[i], pm_structure[(i + 2) % count], k));
            spring_structure.push_back(Spring(pm_structure[i], pm_structure[(i + 3) % count], k));
        }

        outedge = vector<Spring>();
        for (int i = 0; i < count; ++i) {
            outedge.push_back(spring_structure[i * 3]);
        }

        for (int i = 0; i < count; ++i) {
            Spring spring = Spring(pm_structure[i], pm_structure[(i + count / 2) % count], k);
            bool exists = false;
            for (const auto& s : spring_structure) {
                if ((spring.p1 == s.p1 && spring.p2 == s.p2) || (spring.p1 == s.p2 && spring.p2 == s.p1)) {
                    exists = true;
                    break;
                }
            }
            if (!exists) {
                spring_structure.push_back(spring);
            }
        }
        make_innerSprings();
    }

private:
    void make_innerSprings() {
        // Implementation of make_innerSprings() method
    }
};

class Camera {
public:
    Vector2D pos;
    float scale;
    float aspect_ratio;
    Vector2D Camera_ratio;
    int display_width;
    int display_height;

    Camera(Vector2D pos, float scale, int display_width, int display_height)
        : pos(pos), scale(scale), display_width(display_width), display_height(display_height) {
        aspect_ratio = static_cast<float>(display_width) / display_height;
        Camera_ratio = Vector2D(scale, scale / aspect_ratio);
    }

    Vector2D ndc_space(const Vector2D& world_pos) const {
        return Vector2D((world_pos.x - pos.x) / Camera_ratio.x, (world_pos.y - pos.y) / Camera_ratio.y);
    }

    Vector2D screen_space(const Vector2D& world_pos) const {
        Vector2D ndc = ndc_space(world_pos);
        return Vector2D((ndc.x + 1) * display_width / 2, (1 - ((ndc.y + 1) / 2)) * display_height);
    }

    Vector2D screen_to_world(const Vector2D& screen_pos) const {
        Vector2D t = Vector2D((screen_pos.x * 2 / display_width) - 1, 1 - (screen_pos.y * 2 / display_height));
        return Vector2D(t.x * Camera_ratio.x + pos.x, t.y * Camera_ratio.y + pos.y);
    }

    void update_scale(float new_scale) {
        scale = new_scale;
        Camera_ratio = Vector2D(scale, scale / aspect_ratio);
    }
};

class Collinfo {
public:
    PointMass* pm;
    Spring* edge;
    float dist;
    Vector2D normal;
    float interp_value;

    Collinfo(PointMass* pm, Spring* edge, float dist, Vector2D normal, float interp_value)
        : pm(pm), edge(edge), dist(dist), normal(normal), interp_value(interp_value) {}
};

bool check_aabb_collision(const std::array<float, 4> bbox1, const std::array<float, 4> bbox2) {
    if (bbox1[0] < bbox2[1] && bbox1[1] > bbox2[0] && bbox1[2] < bbox2[3] && bbox1[3] > bbox2[2]) {
        return true;
    }
    return false;
}

bool point_to_aabb_check(const std::array<float, 4> bbox, const Vector2D& point) {
    if (point.x > bbox[0] && point.x < bbox[1] && point.y > bbox[2] && point.y < bbox[3]) {
        return true;
    }
    return false;
}

bool horizontal_segment_intersection(const Vector2D& A, const Vector2D& B, const Vector2D& C, const Vector2D& D) {
    float x1 = A.x, y_h = A.y;
    float x2 = B.x;
    float x3 = C.x, y3 = C.y;
    float x4 = D.x, y4 = D.y;

    // Ensure A is to the left of B
    if (x1 > x2) {
        std::swap(x1, x2);
    }

    // Check if y_h is in range of y3 and y4
    if (y_h < std::min(y3, y4) || y_h > std::max(y3, y4)) {
        return false;
    }

    if (y3 == y4) {  // Horizontal line
        return false;
    }

    // Calculate intersection point of line CD with horizontal line y = y_h
    float x_intersection;
    if (x3 == x4) {  // Vertical line
        if (x3 < x1 || x3 > x2) {
            return false;
        }
        x_intersection = x3;
    }
    else {
        // Line equation y = mx + b
        float m = (y4 - y3) / (x4 - x3);
        float b = y3 - m * x3;
        x_intersection = (y_h - b) / m;
    }

    // Check if intersection is within segment bounds
    if (x1 <= x_intersection && x_intersection <= x2) {
        return true;
    }
    return false;
}

bool point_inside_shape(Shape& shape, const Vector2D& point) {
    int count = 0;
    for (const auto& edge : shape.outedge) {
        Vector2D C = edge.p1->pos;
        Vector2D D = edge.p2->pos;
        Vector2D A = point;
        Vector2D B = Vector2D(shape.bbox()[1] + 0.1f, 0);
        if (horizontal_segment_intersection(A, B, C, D)) {
            count++;
        }
    }
    return count % 2 == 1;
}

void resolve_collision(const Collinfo& collinfo) {
    const float tolerance = 0.01f;

    if (collinfo.dist <= tolerance) {
        return;
    }

    PointMass* point_mass = collinfo.pm;
    Spring* edge = collinfo.edge;
    Vector2D normal = collinfo.normal;
    float interp_value = collinfo.interp_value;

    // Calculate the virtual point for the edge
    float total_mass = edge->p1->mass + edge->p2->mass;
    float m_point = point_mass->mass;
    // Distribute virtual point force back to edge points
    float weight_p1 = edge->p1->mass / total_mass;
    float weight_p2 = edge->p2->mass / total_mass;

    // Penetration resolution for point_mass and virtual_point
    Vector2D pmove = normal * (collinfo.dist + tolerance) * (total_mass / (point_mass->mass + total_mass));
    Vector2D emove = normal * (collinfo.dist + tolerance) * (point_mass->mass / (point_mass->mass + total_mass));
    {
        point_mass->pos += pmove;
        edge->p1->pos -= emove * (1 - interp_value) * weight_p1;
        edge->p2->pos -= emove * interp_value * weight_p2;
    }

    // Calculate virtual point position and velocity
    Vector2D virtual_point_pos = (edge->p1->mass * edge->p1->pos + edge->p2->mass * edge->p2->pos) / total_mass;
    Vector2D virtual_point_velocity = (edge->p1->mass * edge->p1->velocity + edge->p2->mass * edge->p2->velocity) / total_mass;

    // Calculate velocity adjustments
    const float restitution = 0.9f;  // Coefficient of restitution
    const float friction_coefficient = 0.5f;  // Coefficient of friction

    // Normal and tangential velocity components
    Vector2D relative_velocity = point_mass->velocity - virtual_point_velocity;
    float normal_velocity = relative_velocity.dot(normal);
    if (normal_velocity > 0.0001f) {
        return;
    }
    Vector2D normal_velocity_vec = normal * normal_velocity;
    Vector2D tangent = Vector2D(-normal.y, normal.x);
    Vector2D tangential_velocity = tangent * relative_velocity.dot(tangent);

    Vector2D impulse = normal_velocity_vec * restitution * point_mass->mass * total_mass / (point_mass->mass + total_mass);

    {
        point_mass->velocity -= impulse / point_mass->mass;
        edge->p1->velocity += impulse * (1 - interp_value) * weight_p1 / edge->p1->mass;
        edge->p2->velocity += impulse * interp_value * weight_p2 / edge->p2->mass;
    }

    Vector2D friction_impulse = tangential_velocity * friction_coefficient * point_mass->mass;

    {
        point_mass->velocity -= friction_impulse / point_mass->mass;
        edge->p1->velocity += friction_impulse * (1 - interp_value) * weight_p1 / edge->p1->mass;
        edge->p2->velocity += friction_impulse * interp_value * weight_p2 / edge->p2->mass;
    }
}

void run_simulation(vector<Shape*>& world, SDL_Renderer* renderer, ImGuiIO* io, Camera* camera,bool threading = false) {
    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
    
    bool running = true;
    SDL_Event event;

    PointMass* dragged_pm = nullptr;
    float dt = 0.001f;
    bool panning = false;
    Vector2D pan_start;
    // Main SDL loop
    int current_time_step = 0;
	float simtime = 0.0f;
    float collision_detection_time = 0.0f;
    float collision_resolution_time = 0.0f;
	float rendertime = 0.0f;
    float deltatime = 0.0f;
    float runtime = 0.0f;
    int count = 0;
	bool use_openmp = threading;
    while (running) {
        static bool changed = false;
        count++;
        auto start = high_resolution_clock::now();
        while (SDL_PollEvent(&event)) {
            ImGui_ImplSDL2_ProcessEvent(&event);
            if (event.type == SDL_QUIT) {
                running = false;
            }
            //mouse event
            if (event.type == SDL_MOUSEBUTTONDOWN) {
                if (event.button.button == SDL_BUTTON_LEFT) {
                    int x, y;
                    SDL_GetMouseState(&x, &y);
                    Vector2D mouse_pos = camera->screen_to_world(Vector2D(x, y));
                    for (auto& shape : world) {
                        for (auto& pm : shape->pm_structure) {
                            if ((pm->pos - mouse_pos).length() < camera->scale / 10) {
                                dragged_pm = pm;
                            }
                        }
                    }
                }
                if (event.button.button == SDL_BUTTON_MIDDLE)
                {
                    panning = true;
                    int x, y;
                    SDL_GetMouseState(&x, &y);
                    pan_start = Vector2D(x, y);
                }
            }
            if (event.type == SDL_MOUSEBUTTONUP) {
                if (event.button.button == SDL_BUTTON_LEFT) {
                    dragged_pm = nullptr;
                }
                if (event.button.button == SDL_BUTTON_MIDDLE)
                {
                    panning = false;
                }
            }

            if (event.type == SDL_MOUSEWHEEL) {
                auto target_scale = max(0.1, camera->scale * (event.wheel.y > 0 ? 1.1 : 0.9));
                camera->update_scale(target_scale);
            }
			if (event.type == SDL_KEYDOWN) {
				if (event.key.keysym.sym == SDLK_o) {
					use_openmp = !use_openmp;
				}
			}
        }
        if (dragged_pm != nullptr) {
            int x, y;
            SDL_GetMouseState(&x, &y);
            Vector2D mouse_pos = camera->screen_to_world(Vector2D(x, y));
            dragged_pm->pos = mouse_pos;
            dragged_pm->update_position_from_vector();
            dragged_pm->velocity = Vector2D(0, 0);
        }
        if (panning) {
            int x, y;
            SDL_GetMouseState(&x, &y);
            Vector2D pan_end = Vector2D(x, y);
            Vector2D delta = camera->screen_to_world(pan_end) - camera->screen_to_world(pan_start);
            camera->pos -= delta;
            pan_start = pan_end;
        }

        // Update forces and point masses - parallelism introduced

        auto simulation_start = high_resolution_clock::now();
		float delta = std::min(0.01f, dt);
        if (use_openmp) {
#pragma omp parallel for 
            for (int i = 0; i < world.size(); ++i) {
                world[i]->update_forces(delta);
            }
        }
        else
        {
            for (auto& shape : world) {
                shape->update_forces(delta);
            }
        }
        auto sim_end = chrono::high_resolution_clock::now();
        chrono::duration<float> duration = sim_end - simulation_start;
		simtime += duration.count();
        
        // everything within this is updating perfectly fine 
        vector<Collinfo> collinfos;

        // Collision detection
        auto start_time_cold = high_resolution_clock::now();        
		//worse possible way imaginable to do this but its late and i need to sleep (code auto compeletion wrote the part after but its late lmao)
        if (use_openmp) {
#pragma omp parallel for
            for (int i = 0; i < world.size(); ++i) {
                auto shape = world[i];
                for (auto& other_shape : world) {
                    if (shape == other_shape) {
                        continue;
                    }
                    auto bbox1 = shape->bbox();
                    auto bbox2 = other_shape->bbox();

                    if (!check_aabb_collision(bbox1, bbox2)) {
                        continue;
                    }

                    for (auto& pm : shape->pm_structure) {
                        if (!point_to_aabb_check(bbox2, pm->pos)) {
                            continue;
                        }

                        if (!point_inside_shape(*other_shape, pm->pos)) {
                            continue;
                        }
                        Collinfo collinfo = Collinfo(pm, nullptr, INFINITY, Vector2D(0, 0), 0);
                        for (auto& edge : other_shape->outedge) {
                            // Calculate distance between point mass and edge
                            Vector2D A = edge.p1->pos;
                            Vector2D B = edge.p2->pos;
                            Vector2D C = pm->pos;
                            Vector2D AB = B - A;
                            Vector2D AC = C - A;
                            auto length_AB = AB.length();
                            float ab2 = AB.dot(AB);
                            float acab = AC.dot(AB);
                            auto dist = abs(AB.cross(AC)) / length_AB;
                            auto AB_dir = AB / length_AB;
                            if (dist < collinfo.dist) {
                                collinfo.dist = dist;
                                float t = acab / ab2;
                                t = max(0.0f, min(1.0f, t));
                                collinfo.normal = Vector2D(-AB_dir.y, AB_dir.x);
                                if (AC.dot(collinfo.normal) > 0) {
                                    collinfo.normal = -1 * collinfo.normal;
                                }
                                collinfo.interp_value = t;
                                collinfo.edge = &edge;
                            }
                        }
                        if (collinfo.edge != nullptr) {
#pragma omp critical
                            collinfos.push_back(collinfo);
                        }
                    }

                }
            }
        }
        else {
            for (int i = 0; i < world.size(); ++i) {
                auto shape = world[i];
                for (auto& other_shape : world) {
                    if (shape == other_shape) {
                        continue;
                    }
                    auto bbox1 = shape->bbox();
                    auto bbox2 = other_shape->bbox();

                    if (!check_aabb_collision(bbox1, bbox2)) {
                        continue;
                    }

                    for (auto& pm : shape->pm_structure) {
                        if (!point_to_aabb_check(bbox2, pm->pos)) {
                            continue;
                        }

                        if (!point_inside_shape(*other_shape, pm->pos)) {
                            continue;
                        }
                        Collinfo collinfo = Collinfo(pm, nullptr, INFINITY, Vector2D(0, 0), 0);
                        for (auto& edge : other_shape->outedge) {
                            // Calculate distance between point mass and edge
                            Vector2D A = edge.p1->pos;
                            Vector2D B = edge.p2->pos;
                            Vector2D C = pm->pos;
                            Vector2D AB = B - A;
                            Vector2D AC = C - A;
                            auto length_AB = AB.length();
                            float ab2 = AB.dot(AB);
                            float acab = AC.dot(AB);
                            auto dist = abs(AB.cross(AC)) / length_AB;
                            auto AB_dir = AB / length_AB;
                            if (dist < collinfo.dist) {
                                collinfo.dist = dist;
                                float t = acab / ab2;
                                t = max(0.0f, min(1.0f, t));
                                collinfo.normal = Vector2D(-AB_dir.y, AB_dir.x);
                                if (AC.dot(collinfo.normal) > 0) {
                                    collinfo.normal = -1 * collinfo.normal;
                                }
                                collinfo.interp_value = t;
                                collinfo.edge = &edge;
                            }
                        }
                        if (collinfo.edge != nullptr) {
                            collinfos.push_back(collinfo);
                        }
                    }

                }
            }
        }
        auto end_time_cold = chrono::high_resolution_clock::now();
        chrono::duration<float> collision_duration = end_time_cold - start_time_cold;
        collision_detection_time += collision_duration.count();
       
        // Clear screen
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 1);
        SDL_RenderClear(renderer);

        // Collision resolution
        auto start_time_colr = high_resolution_clock::now();
        if (use_openmp) {
            for (int i = 0;i < collinfos.size();i++) {
                resolve_collision(collinfos[i]);
            }
        }
        else
        {
            for (int i = 0;i < collinfos.size();i++) {
                resolve_collision(collinfos[i]);
            }
        }
        auto end_time_colr = chrono::high_resolution_clock::now();
        chrono::duration<float> collision_res_duration = end_time_colr - start_time_colr;
        collision_resolution_time += collision_res_duration.count();

        // RENDERING
        auto start_time_rendering = high_resolution_clock::now();
        // black color for springs
        for (auto& shape : world) {
            SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
            for (auto& spring : shape->spring_structure) {
                //use camera for transformation
                Vector2D p1 = camera->screen_space(spring.p1->pos);
                Vector2D p2 = camera->screen_space(spring.p2->pos);
                SDL_RenderDrawLine(renderer, p1.x, p1.y, p2.x, p2.y);
                //cout << "Y position: " << spring.p1->position.y << endl; 
            }
            SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255);
        }

        // Draw point masses as larger red dots
        SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255); // red color for point masses
        int radius = 2;
        for (auto& shape : world) {
            for (auto& pm : shape->pm_structure) {
                //use camera for transformation
                Vector2D p = camera->screen_space(pm->pos);
                SDL_Rect rect{ static_cast<int>(p.x - radius), static_cast<int>(p.y - radius), 2 * radius, 2 * radius };
                SDL_RenderFillRect(renderer,&rect);
            }
        }

        //imgui
        {
            ImGui_ImplSDLRenderer2_NewFrame();
            ImGui_ImplSDL2_NewFrame();
            ImGui::NewFrame();

            //make window non movable nad put it on top left
			ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Always);
            //and non resizeable 
			ImGui::SetNextWindowSize(ImVec2(285, 200), ImGuiCond_Always);
			

            ImGui::Begin("Softbody Simulation");
            ImGui::Text("Mode: %s %0.2f s", use_openmp ? "OpenMP" : "Sequential",runtime);
            if (ImGui::Button("Change"))
            {
                changed = true;
                use_openmp = !use_openmp;
            }
            ImGui::Text("Average FPS/dt: %.02f/%.5f ms", ((count) / deltatime), (deltatime*1000 / count));
            ImGui::Text("Simulation Time: %.5f ms", simtime*1000 / count);
            ImGui::Text("Total Collision Time: %.5f ms", (collision_resolution_time + collision_detection_time) * 1000 / count);
            ImGui::Text("Collision Detection Time: %.5f ms", collision_detection_time*1000 / count);
            ImGui::Text("Collision Resolution Time: %.5f ms", collision_resolution_time * 1000 / count);            
            ImGui::Text("Render Time: %.5f ms", rendertime * 1000 / count);
            ImGui::Text("Number of Shapes: %d", world.size());
            ImGui::End();
            ImGui::Render();
            SDL_RenderSetScale(renderer, io->DisplayFramebufferScale.x, io->DisplayFramebufferScale.y);
            SDL_SetRenderDrawColor(renderer, (Uint8)(clear_color.x * 255), (Uint8)(clear_color.y * 255), (Uint8)(clear_color.z * 255), (Uint8)(clear_color.w * 255));
            ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData(), renderer);
        }


        // Present the renderer
        SDL_RenderPresent(renderer);
        auto end_time_rendering = chrono::high_resolution_clock::now();
        chrono::duration<float> render_duration = end_time_rendering - start_time_rendering;
        rendertime += render_duration.count();

        if (count > 1000)
        {
            float target = 100;
            simtime *= target / count;
            collision_detection_time *= target / count;
            collision_resolution_time *= target / count;
            rendertime *= target / count;
            deltatime *= target / count;
            count = target;
        }
        if (changed)
        {
			changed = false;
            simtime = 0;
            collision_detection_time =0;
            collision_resolution_time = 0;
            rendertime = 0;
            deltatime = 0;
            count = 1;
            runtime = 0;
        }

		auto end = high_resolution_clock::now();
        chrono::duration<float> duration_all = end - start;
        dt = duration_all.count();
        deltatime += dt;
		runtime += delta;
    }

    //std::cout << "Total simulation update force time when run with OpenMP for simulation with " << num_time_steps << " steps and " << world.size() << " shapes in simulation: " << total_force_update_time << " milliseconds." << endl;
    ////std::cout << "Total simulation update force time when run sequentially for simulation with " << num_time_steps << " steps and " << world.size() << " shapes in simulation: " << total_force_update_time << " milliseconds." << endl;
    //std::cout << "Sequential collision detection time : " << total_collision_detectio_time << " milliseconds" << endl;
    //std::cout << "Total parallelizable regions time : " << total_parallelizable_time << " milliseconds" << endl;
}

int SDL_main(int argc, char* argv[]) {
    // Initialize SDL
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        std::cerr << "SDL could not initialize! SDL_Error: " << SDL_GetError() << std::endl;
        return -1;
    }

#ifdef SDL_HINT_IME_SHOW_UI
    SDL_SetHint(SDL_HINT_IME_SHOW_UI, "1");
#endif

    // Create window and renderer
    SDL_Window* window = SDL_CreateWindow("Physics Simulation", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, WINDOW_WIDTH, WINDOW_HEIGHT, SDL_WINDOW_SHOWN);
    if (!window) {
        std::cerr << "Window could not be created! SDL_Error: " << SDL_GetError() << std::endl;
        SDL_Quit();
        return -1;
    }

    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (!renderer) {
        std::cerr << "Renderer could not be created! SDL_Error: " << SDL_GetError() << std::endl;
        SDL_DestroyWindow(window);
        SDL_Quit();
        return -1;
    }

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;

    ImGui::StyleColorsDark();

    // Setup Platform/Renderer backends
    ImGui_ImplSDL2_InitForSDLRenderer(window, renderer);
    ImGui_ImplSDLRenderer2_Init(renderer);


    // Create simulation world
    vector<Shape*> world;
    // int num_shapes = 20;
    // for (int i = 0; i < num_shapes; i++) {
    //     // Randomize shape type
    //     if (i % 2 == 0) { 
    //         float x = static_cast<float>(rand() % 20 - 10);  // Random x between -10 and 10
    //         float y = static_cast<float>(rand() % 5);  
    //         float size = 0.25;
    //         int mass = rand() % 10 + 10;  
    //         world.push_back(new Square(x, y, size, mass));
    //     }
    //     else { 
    //         float x = static_cast<float>(rand() % 20 - 10); 
    //         float y = static_cast<float>(rand() % 5); 
    //         float radius = 0.25;
    //         int points = 11;
    //         int mass = rand() % 10 + 10;  
    //         world.push_back(new Circle(x, y, radius, points, mass));
    //     }
    // }
	
    bool use_multithreading = true;
    int num_shapes = 8;

    int rows = static_cast<int>(ceil(sqrt(num_shapes)));
    int cols = static_cast<int>(ceil(static_cast<float>(num_shapes) / rows));
    float spacing = 3.0f; // Adjust spacing as needed
    for(int i = 0; i < rows; ++i){
        for(int j = 0; j < cols; ++j){
            if(world.size() >= num_shapes){
                break;
            }
            float x = j * spacing;
            float y = i * spacing;
            if( (i + j) % 2 == 0 ){
                world.push_back(new Square(x, y, 1.0f, 50.0f, 1.0f));
            }
            else{
                world.push_back(new Circle(x, y, 1.0f, 11, 100.0f, 1.0f));
            }
        }
    }
    Camera Main_camera = Camera(Vector2D(5, 5), 15.0f, WINDOW_WIDTH, WINDOW_HEIGHT);

    // Run the simulation with SDL rendering
    run_simulation(world, renderer,  &io, &Main_camera, use_multithreading);

    // Cleanup and close SDL
    for (auto& shape : world) {
        delete shape;
    }

    ImGui_ImplSDLRenderer2_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();


    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}

