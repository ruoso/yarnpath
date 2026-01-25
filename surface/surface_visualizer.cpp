#include "surface_visualizer.hpp"
#include "logging.hpp"

#ifdef YARNPATH_HAS_VISUALIZATION

#include <GLFW/glfw3.h>
#include <cmath>

namespace yarnpath {

// Camera state
struct Camera {
    float distance = 20.0f;
    float yaw = 0.0f;      // Rotation around Y axis
    float pitch = 0.3f;    // Rotation around X axis
    float target_x = 0.0f;
    float target_y = 0.0f;
    float target_z = 0.0f;

    void apply() const {
        glTranslatef(0, 0, -distance);
        glRotatef(pitch * 180.0f / 3.14159f, 1, 0, 0);
        glRotatef(yaw * 180.0f / 3.14159f, 0, 1, 0);
        glTranslatef(-target_x, -target_y, -target_z);
    }
};

// Global state for callbacks
static Camera g_camera;
static bool g_mouse_dragging = false;
static double g_last_mouse_x = 0;
static double g_last_mouse_y = 0;
static bool g_paused = false;
static float g_rotation_speed = 0.5f;
static float g_zoom_speed = 1.1f;

static void mouse_button_callback(GLFWwindow* window, int button, int action, int mods) {
    (void)window;
    (void)mods;
    if (button == GLFW_MOUSE_BUTTON_LEFT) {
        g_mouse_dragging = (action == GLFW_PRESS);
    }
}

static void cursor_position_callback(GLFWwindow* window, double xpos, double ypos) {
    (void)window;
    if (g_mouse_dragging) {
        float dx = static_cast<float>(xpos - g_last_mouse_x);
        float dy = static_cast<float>(ypos - g_last_mouse_y);
        g_camera.yaw += dx * g_rotation_speed * 0.01f;
        g_camera.pitch += dy * g_rotation_speed * 0.01f;
        // Clamp pitch
        if (g_camera.pitch > 1.5f) g_camera.pitch = 1.5f;
        if (g_camera.pitch < -1.5f) g_camera.pitch = -1.5f;
    }
    g_last_mouse_x = xpos;
    g_last_mouse_y = ypos;
}

static void scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
    (void)window;
    (void)xoffset;
    if (yoffset > 0) {
        g_camera.distance /= g_zoom_speed;
    } else if (yoffset < 0) {
        g_camera.distance *= g_zoom_speed;
    }
}

static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    (void)scancode;
    (void)mods;
    if (action == GLFW_PRESS) {
        if (key == GLFW_KEY_ESCAPE || key == GLFW_KEY_Q) {
            glfwSetWindowShouldClose(window, GLFW_TRUE);
        } else if (key == GLFW_KEY_SPACE) {
            g_paused = !g_paused;
        } else if (key == GLFW_KEY_R) {
            // Reset camera
            g_camera.yaw = 0.0f;
            g_camera.pitch = 0.3f;
        }
    }
}

static void fit_camera_to_graph(const SurfaceGraph& graph, Camera& camera, float margin = 1.2f) {
    if (graph.node_count() == 0) return;

    // Compute bounding box
    Vec3 min_pt = graph.node(0).position;
    Vec3 max_pt = graph.node(0).position;

    for (const auto& node : graph.nodes()) {
        min_pt.x = std::min(min_pt.x, node.position.x);
        min_pt.y = std::min(min_pt.y, node.position.y);
        min_pt.z = std::min(min_pt.z, node.position.z);
        max_pt.x = std::max(max_pt.x, node.position.x);
        max_pt.y = std::max(max_pt.y, node.position.y);
        max_pt.z = std::max(max_pt.z, node.position.z);
    }

    // Center camera on bounding box center
    camera.target_x = (min_pt.x + max_pt.x) * 0.5f;
    camera.target_y = (min_pt.y + max_pt.y) * 0.5f;
    camera.target_z = (min_pt.z + max_pt.z) * 0.5f;

    // Set distance to fit entire bounding box
    float size_x = max_pt.x - min_pt.x;
    float size_y = max_pt.y - min_pt.y;
    float size_z = max_pt.z - min_pt.z;
    float max_size = std::max({size_x, size_y, size_z});

    // Distance needed to fit object in view (assuming ~45 degree FOV)
    camera.distance = max_size * margin;
    if (camera.distance < 1.0f) camera.distance = 1.0f;
}

static void draw_sphere(float x, float y, float z, float radius, int segments = 8) {
    glPushMatrix();
    glTranslatef(x, y, z);

    for (int i = 0; i < segments; ++i) {
        float lat0 = 3.14159f * (-0.5f + static_cast<float>(i) / segments);
        float lat1 = 3.14159f * (-0.5f + static_cast<float>(i + 1) / segments);
        float z0 = std::sin(lat0) * radius;
        float z1 = std::sin(lat1) * radius;
        float r0 = std::cos(lat0) * radius;
        float r1 = std::cos(lat1) * radius;

        glBegin(GL_QUAD_STRIP);
        for (int j = 0; j <= segments; ++j) {
            float lng = 2.0f * 3.14159f * static_cast<float>(j) / segments;
            float x_coord = std::cos(lng);
            float y_coord = std::sin(lng);

            glNormal3f(x_coord * r0, y_coord * r0, z0);
            glVertex3f(x_coord * r0, y_coord * r0, z0);
            glNormal3f(x_coord * r1, y_coord * r1, z1);
            glVertex3f(x_coord * r1, y_coord * r1, z1);
        }
        glEnd();
    }

    glPopMatrix();
}

static void draw_line(float x1, float y1, float z1, float x2, float y2, float z2) {
    glBegin(GL_LINES);
    glVertex3f(x1, y1, z1);
    glVertex3f(x2, y2, z2);
    glEnd();
}

static void render_graph(const SurfaceGraph& graph, const VisualizerConfig& config) {
    // Draw edges first (behind nodes)
    glDisable(GL_LIGHTING);
    glLineWidth(2.0f);

    if (config.show_continuity) {
        glColor3f(0.2f, 0.9f, 0.2f);  // Green for continuity
        for (const auto& edge : graph.edges()) {
            if (edge.type == EdgeType::YarnContinuity) {
                const auto& a = graph.node(edge.node_a).position;
                const auto& b = graph.node(edge.node_b).position;
                draw_line(a.x, a.y, a.z, b.x, b.y, b.z);
            }
        }
    }

    if (config.show_passthrough) {
        glColor3f(0.9f, 0.2f, 0.2f);  // Red for passthrough
        for (const auto& edge : graph.edges()) {
            if (edge.type == EdgeType::PassThrough) {
                const auto& a = graph.node(edge.node_a).position;
                const auto& b = graph.node(edge.node_b).position;
                draw_line(a.x, a.y, a.z, b.x, b.y, b.z);
            }
        }
    }

    // Draw nodes
    glEnable(GL_LIGHTING);
    for (const auto& node : graph.nodes()) {
        if (config.color_by_type) {
            if (node.forms_loop) {
                glColor3f(0.2f, 0.4f, 0.8f);  // Blue for loops
            } else {
                glColor3f(0.8f, 0.5f, 0.2f);  // Orange for connectors
            }
        } else {
            glColor3f(0.7f, 0.7f, 0.7f);
        }
        draw_sphere(node.position.x, node.position.y, node.position.z, config.node_size);
    }
}

static void setup_lighting() {
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);

    GLfloat light_pos[] = {1.0f, 1.0f, 1.0f, 0.0f};
    GLfloat light_ambient[] = {0.2f, 0.2f, 0.2f, 1.0f};
    GLfloat light_diffuse[] = {0.8f, 0.8f, 0.8f, 1.0f};

    glLightfv(GL_LIGHT0, GL_POSITION, light_pos);
    glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
}

VisualizerResult visualize_relaxation(
    SurfaceGraph& graph,
    const YarnProperties& yarn,
    const SolveConfig& solve_config,
    const VisualizerConfig& viz_config) {

    auto log = yarnpath::logging::get_logger();
    VisualizerResult result;

    // Initialize GLFW
    if (!glfwInit()) {
        log->error("Failed to initialize GLFW");
        return result;
    }

    // Window hints for visibility
    glfwWindowHint(GLFW_VISIBLE, GLFW_TRUE);
    glfwWindowHint(GLFW_FOCUSED, GLFW_TRUE);

    // Create window
    GLFWwindow* window = glfwCreateWindow(
        viz_config.window_width,
        viz_config.window_height,
        viz_config.window_title.c_str(),
        nullptr, nullptr);

    if (!window) {
        log->error("Failed to create GLFW window");
        glfwTerminate();
        return result;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);  // Enable vsync

    // Show window and ensure it's visible
    glfwShowWindow(window);
    glfwFocusWindow(window);

    // Do an initial
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glfwSwapBuffers(window);
    glfwPollEvents();

    // Set up callbacks
    glfwSetMouseButtonCallback(window, mouse_button_callback);
    glfwSetCursorPosCallback(window, cursor_position_callback);
    glfwSetScrollCallback(window, scroll_callback);
    glfwSetKeyCallback(window, key_callback);

    // Initialize camera
    g_camera.distance = viz_config.camera_distance;
    g_rotation_speed = viz_config.rotation_speed;
    g_zoom_speed = viz_config.zoom_speed;
    g_paused = !viz_config.auto_run;

    // Initial camera fit
    fit_camera_to_graph(graph, g_camera);

    // Set up OpenGL
    glEnable(GL_DEPTH_TEST);
    setup_lighting();
    glClearColor(0.1f, 0.1f, 0.15f, 1.0f);

    int iteration = 0;
    float energy = graph.compute_energy();

    log->info("Visualizer started. Controls: mouse drag=rotate, scroll=zoom, space=pause, q=quit");

    // Main loop
    while (!glfwWindowShouldClose(window)) {
        // Handle window resize
        int width, height;
        glfwGetFramebufferSize(window, &width, &height);
        glViewport(0, 0, width, height);

        // Set up projection
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        float aspect = static_cast<float>(width) / static_cast<float>(height);
        float fov = 45.0f * 3.14159f / 180.0f;
        float near = 0.1f;
        float far = 1000.0f;
        float top = near * std::tan(fov / 2.0f);
        float right = top * aspect;
        glFrustum(-right, right, -top, top, near, far);

        // Auto-fit camera to show all nodes (if enabled and not dragging)
        if (viz_config.auto_fit && !g_mouse_dragging) {
            fit_camera_to_graph(graph, g_camera);
        }

        // Set up modelview
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        g_camera.apply();

        // Clear
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Run solver steps if not paused
        if (!g_paused) {
            for (int i = 0; i < viz_config.steps_per_frame; ++i) {
                SurfaceSolver::step(graph, yarn, solve_config);
                iteration++;

                // Check convergence
                float new_energy = graph.compute_energy();
                if (std::abs(new_energy - energy) < solve_config.convergence_threshold) {
                    g_paused = true;
                    log->info("Converged at iteration {}", iteration);
                    break;
                }
                energy = new_energy;

                if (iteration >= solve_config.max_iterations) {
                    g_paused = true;
                    log->info("Max iterations reached");
                    break;
                }
            }
        }

        // Render
        render_graph(graph, viz_config);

        // Swap buffers and poll events
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    result.completed = true;
    result.total_iterations = iteration;
    result.final_energy = energy;

    glfwDestroyWindow(window);
    glfwTerminate();

    return result;
}

bool visualization_available() {
    return true;
}

}  // namespace yarnpath

#else  // YARNPATH_HAS_VISUALIZATION not defined

namespace yarnpath {

VisualizerResult visualize_relaxation(
    SurfaceGraph&,
    const YarnProperties&,
    const SolveConfig&,
    const VisualizerConfig&) {

    auto log = yarnpath::logging::get_logger();
    log->error("Visualization not available - compile with GLFW and OpenGL");
    return VisualizerResult{};
}

bool visualization_available() {
    return false;
}

}  // namespace yarnpath

#endif  // YARNPATH_HAS_VISUALIZATION
