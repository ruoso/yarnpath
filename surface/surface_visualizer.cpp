#include "surface_visualizer.hpp"
#include "logging.hpp"

#ifdef YARNPATH_HAS_VISUALIZATION

#include <GLFW/glfw3.h>
#include <cmath>
#include <vector>

namespace yarnpath {

// Snapshot of node positions at a point in time
struct Snapshot {
    int iteration;
    float energy;
    std::vector<Vec3> positions;
    std::vector<Vec3> velocities;
};

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

// History navigation
static std::vector<Snapshot> g_snapshots;
static int g_current_snapshot = -1;  // -1 means live view
static bool g_viewing_history = false;
static bool g_request_fit_camera = false;  // Request camera fit on next frame

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

static void navigate_history(int delta) {
    if (g_snapshots.empty()) return;

    if (!g_viewing_history) {
        // Enter history mode at the latest snapshot
        g_viewing_history = true;
        g_current_snapshot = static_cast<int>(g_snapshots.size()) - 1;
        g_paused = true;
    }

    g_current_snapshot += delta;

    // Clamp to valid range
    if (g_current_snapshot < 0) {
        g_current_snapshot = 0;
    }
    if (g_current_snapshot >= static_cast<int>(g_snapshots.size())) {
        g_current_snapshot = static_cast<int>(g_snapshots.size()) - 1;
    }
}

static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    (void)scancode;
    (void)mods;
    if (action == GLFW_PRESS || action == GLFW_REPEAT) {
        if (key == GLFW_KEY_ESCAPE || key == GLFW_KEY_Q) {
            glfwSetWindowShouldClose(window, GLFW_TRUE);
        } else if (key == GLFW_KEY_SPACE) {
            if (g_viewing_history) {
                // Exit history mode, resume live
                g_viewing_history = false;
                g_current_snapshot = -1;
            }
            g_paused = !g_paused;
        } else if (key == GLFW_KEY_R) {
            // Reset camera (orientation and request fit)
            g_camera.yaw = 0.0f;
            g_camera.pitch = 0.3f;
            g_request_fit_camera = true;
        } else if (key == GLFW_KEY_LEFT) {
            // Previous frame
            navigate_history(-1);
        } else if (key == GLFW_KEY_RIGHT) {
            // Next frame
            navigate_history(1);
        } else if (key == GLFW_KEY_PAGE_UP) {
            // Back 30 frames
            navigate_history(-30);
        } else if (key == GLFW_KEY_PAGE_DOWN) {
            // Forward 30 frames
            navigate_history(30);
        } else if (key == GLFW_KEY_HOME) {
            // Go to first snapshot
            if (!g_snapshots.empty()) {
                g_viewing_history = true;
                g_current_snapshot = 0;
                g_paused = true;
            }
        } else if (key == GLFW_KEY_END) {
            // Go to last snapshot / live view
            g_viewing_history = false;
            g_current_snapshot = -1;
        }
    }
}

static Snapshot take_snapshot(const SurfaceGraph& graph, int iteration, float energy) {
    Snapshot snap;
    snap.iteration = iteration;
    snap.energy = energy;
    snap.positions.reserve(graph.node_count());
    snap.velocities.reserve(graph.node_count());

    for (const auto& node : graph.nodes()) {
        snap.positions.push_back(node.position);
        snap.velocities.push_back(node.velocity);
    }

    return snap;
}

static void apply_snapshot(SurfaceGraph& graph, const Snapshot& snap) {
    auto& nodes = graph.nodes();
    for (size_t i = 0; i < nodes.size() && i < snap.positions.size(); ++i) {
        nodes[i].position = snap.positions[i];
        nodes[i].velocity = snap.velocities[i];
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

    // Initialize history
    g_snapshots.clear();
    g_current_snapshot = -1;
    g_viewing_history = false;

    // Initial camera fit
    fit_camera_to_graph(graph, g_camera);

    // Set up OpenGL
    glEnable(GL_DEPTH_TEST);
    setup_lighting();
    glClearColor(0.1f, 0.1f, 0.15f, 1.0f);

    int iteration = 0;
    int frame_count = 0;
    float energy = graph.compute_energy();
    int last_logged_snapshot = -1;

    // Take initial snapshot
    g_snapshots.push_back(take_snapshot(graph, iteration, energy));

    log->info("Visualizer started. Controls:");
    log->info("  mouse drag=rotate, scroll=zoom");
    log->info("  space=pause/resume, q=quit, r=reset camera");
    log->info("  left/right=frame by frame, pgup/pgdn=30 frames");
    log->info("  home=first frame, end=live view");

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

        // Fit camera when requested (R key)
        if (g_request_fit_camera) {
            fit_camera_to_graph(graph, g_camera);
            g_request_fit_camera = false;
        }

        // Set up modelview
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        g_camera.apply();

        // Clear
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Run solver steps if not paused and not viewing history
        if (!g_paused && !g_viewing_history) {
            float prev_energy = energy;

            for (int i = 0; i < viz_config.steps_per_frame; ++i) {
                SurfaceSolver::step(graph, yarn, solve_config);
                iteration++;

                // Check convergence
                float new_energy = graph.compute_energy();
                if (std::abs(new_energy - energy) < solve_config.convergence_threshold) {
                    g_paused = true;
                    log->info("Converged at iteration {}, energy={}", iteration, new_energy);
                    break;
                }
                energy = new_energy;

                if (iteration >= solve_config.max_iterations) {
                    g_paused = true;
                    log->info("Max iterations reached, energy={}", energy);
                    break;
                }
            }

            // Log progress periodically (every 100 iterations)
            if (iteration % 100 == 0) {
                float delta = energy - prev_energy;
                log->debug("Iteration {}: energy={:.6f}, delta={:.6f}", iteration, energy, delta);
            }

            // Take snapshot at configured interval
            frame_count++;
            if (frame_count % viz_config.snapshot_interval == 0) {
                if (static_cast<int>(g_snapshots.size()) < viz_config.max_snapshots) {
                    g_snapshots.push_back(take_snapshot(graph, iteration, energy));
                }
            }
        }

        // If viewing history, apply the snapshot to the graph for rendering
        if (g_viewing_history && g_current_snapshot >= 0 &&
            g_current_snapshot < static_cast<int>(g_snapshots.size())) {
            apply_snapshot(graph, g_snapshots[g_current_snapshot]);

            // Log frame info when snapshot changes
            if (g_current_snapshot != last_logged_snapshot) {
                const auto& snap = g_snapshots[g_current_snapshot];
                log->info("Frame {}/{}: iteration={}, energy={}",
                         g_current_snapshot + 1, g_snapshots.size(),
                         snap.iteration, snap.energy);
                last_logged_snapshot = g_current_snapshot;
            }
        }

        // Render
        render_graph(graph, viz_config);

        // Swap buffers and poll events
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // If we were viewing history, restore the final state
    if (g_viewing_history && !g_snapshots.empty()) {
        apply_snapshot(graph, g_snapshots.back());
    }

    result.completed = true;
    result.total_iterations = iteration;
    result.final_energy = energy;

    // Clean up global state
    g_snapshots.clear();
    g_current_snapshot = -1;
    g_viewing_history = false;

    glfwDestroyWindow(window);
    glfwTerminate();

    log->info("Visualization ended. {} snapshots recorded.", frame_count + 1);

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
