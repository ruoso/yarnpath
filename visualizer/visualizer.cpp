#include "visualizer.hpp"
#include "logging.hpp"
#include <geometry/geometry_builder.hpp>

#ifdef YARNPATH_HAS_VISUALIZATION

#include <GLFW/glfw3.h>
#include <cmath>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>

namespace yarnpath {

// Snapshot of node positions at a point in time
struct Snapshot {
    int iteration;
    float energy;
    std::vector<Vec3> positions;
    std::vector<Vec3> velocities;
};

// Geometry snapshot during build process
struct GeometrySnapshot {
    SegmentId segment_id;
    std::string description;
    BezierSpline spline;  // Accumulated spline at this point
};

// Local coordinate frame at a point along a curve
struct CurveFrame {
    Vec3 position;
    Vec3 tangent;
    Vec3 normal;
    Vec3 binormal;
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
static bool g_mouse_panning = false;  // true when shift is held during drag
static double g_last_mouse_x = 0;
static double g_last_mouse_y = 0;
static bool g_paused = false;
static float g_rotation_speed = 0.5f;
static float g_zoom_speed = 1.1f;
static float g_pan_speed = 0.01f;

// History navigation
static std::vector<Snapshot> g_snapshots;
static int g_current_snapshot = -1;  // -1 means live view
static bool g_viewing_history = false;
static bool g_request_fit_camera = false;  // Request camera fit on next frame

// Display toggles
static bool g_show_nodes = true;
static bool g_show_geometry = true;

// Geometry build snapshots
static std::vector<GeometrySnapshot> g_geometry_snapshots;
static int g_current_geometry_snapshot = -1;  // -1 means show full geometry
static bool g_viewing_geometry_history = false;
static int g_last_logged_geometry_snapshot = -2;  // Track for logging

static void mouse_button_callback(GLFWwindow* window, int button, int action, int mods) {
    (void)window;
    if (button == GLFW_MOUSE_BUTTON_LEFT) {
        g_mouse_dragging = (action == GLFW_PRESS);
        g_mouse_panning = (action == GLFW_PRESS) && (mods & GLFW_MOD_SHIFT);
    }
}

static void cursor_position_callback(GLFWwindow* window, double xpos, double ypos) {
    (void)window;
    if (g_mouse_dragging) {
        float dx = static_cast<float>(xpos - g_last_mouse_x);
        float dy = static_cast<float>(ypos - g_last_mouse_y);
        
        if (g_mouse_panning) {
            // Pan: move camera target in screen-aligned directions
            // Calculate right and up vectors based on current camera orientation
            float cos_yaw = std::cos(g_camera.yaw);
            float sin_yaw = std::sin(g_camera.yaw);
            float cos_pitch = std::cos(g_camera.pitch);
            float sin_pitch = std::sin(g_camera.pitch);
            
            // Right vector (perpendicular to view direction, always horizontal)
            float right_x = cos_yaw;
            float right_z = -sin_yaw;
            
            // Up vector (perpendicular to both view and right, accounts for pitch)
            // When pitch=0, up is (0,1,0). As pitch increases, up tilts.
            float up_x = -sin_yaw * sin_pitch;
            float up_y = cos_pitch;
            float up_z = -cos_yaw * sin_pitch;
            
            // Scale pan by distance for consistent feel
            float pan_scale = g_camera.distance * g_pan_speed;
            
            // Move target: right for +dx, up for -dy
            g_camera.target_x -= dx * right_x * pan_scale;
            g_camera.target_z -= dx * right_z * pan_scale;
            g_camera.target_x += dy * up_x * pan_scale;
            g_camera.target_y += dy * up_y * pan_scale;
            g_camera.target_z += dy * up_z * pan_scale;
        } else {
            // Rotate
            g_camera.yaw += dx * g_rotation_speed * 0.01f;
            g_camera.pitch += dy * g_rotation_speed * 0.01f;
            // Clamp pitch
            if (g_camera.pitch > 1.5f) g_camera.pitch = 1.5f;
            if (g_camera.pitch < -1.5f) g_camera.pitch = -1.5f;
        }
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

static void navigate_geometry_history(int delta) {
    if (g_geometry_snapshots.empty()) return;

    if (!g_viewing_geometry_history) {
        // Enter geometry history mode at the first snapshot
        g_viewing_geometry_history = true;
        g_current_geometry_snapshot = 0;
    }

    g_current_geometry_snapshot += delta;

    // Clamp to valid range
    if (g_current_geometry_snapshot < 0) {
        g_current_geometry_snapshot = 0;
    }
    if (g_current_geometry_snapshot >= static_cast<int>(g_geometry_snapshots.size())) {
        // Going past the end exits geometry history mode (show full geometry)
        g_viewing_geometry_history = false;
        g_current_geometry_snapshot = -1;
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
        } else if (key == GLFW_KEY_N) {
            // Toggle node display
            g_show_nodes = !g_show_nodes;
        } else if (key == GLFW_KEY_G) {
            // Toggle geometry display
            g_show_geometry = !g_show_geometry;
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
        } else if (key == GLFW_KEY_B) {
            // Step back in geometry build history
            navigate_geometry_history(-1);
        } else if (key == GLFW_KEY_F) {
            // Step forward in geometry build history
            navigate_geometry_history(1);
        } else if (key == GLFW_KEY_0) {
            // Reset geometry view to full geometry
            g_viewing_geometry_history = false;
            g_current_geometry_snapshot = -1;
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

static void draw_sphere(float x, float y, float z, float compressed_radius, int segments = 8) {
    glPushMatrix();
    glTranslatef(x, y, z);

    for (int i = 0; i < segments; ++i) {
        float lat0 = 3.14159f * (-0.5f + static_cast<float>(i) / segments);
        float lat1 = 3.14159f * (-0.5f + static_cast<float>(i + 1) / segments);
        float z0 = std::sin(lat0) * compressed_radius;
        float z1 = std::sin(lat1) * compressed_radius;
        float r0 = std::cos(lat0) * compressed_radius;
        float r1 = std::cos(lat1) * compressed_radius;

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

// Render XYZ orientation gizmo in the corner of the screen
static void render_axis_gizmo(int window_width, int window_height, const Camera& camera) {
    int gizmo_size = 80;  // Size of the gizmo viewport in pixels
    int margin = 10;      // Margin from corner
    
    // Save current state
    glPushAttrib(GL_ALL_ATTRIB_BITS);
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    
    // Set up viewport in bottom-left corner
    glViewport(margin, margin, gizmo_size, gizmo_size);
    
    // Set up orthographic projection
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    float ortho_size = 1.5f;
    glOrtho(-ortho_size, ortho_size, -ortho_size, ortho_size, -10.0f, 10.0f);
    
    // Apply only the rotation part of the camera (no translation or zoom)
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glRotatef(camera.pitch * 180.0f / 3.14159f, 1, 0, 0);
    glRotatef(camera.yaw * 180.0f / 3.14159f, 0, 1, 0);
    
    // Clear depth buffer for this region so gizmo is always visible
    glClear(GL_DEPTH_BUFFER_BIT);
    
    // Disable lighting for simple colored lines
    glDisable(GL_LIGHTING);
    glLineWidth(2.0f);
    
    float axis_length = 1.0f;
    float arrow_size = 0.15f;
    
    // X axis - Red
    glColor3f(1.0f, 0.2f, 0.2f);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(axis_length, 0, 0);
    glEnd();
    // Arrow head
    glBegin(GL_TRIANGLES);
    glVertex3f(axis_length, 0, 0);
    glVertex3f(axis_length - arrow_size, arrow_size * 0.5f, 0);
    glVertex3f(axis_length - arrow_size, -arrow_size * 0.5f, 0);
    glVertex3f(axis_length, 0, 0);
    glVertex3f(axis_length - arrow_size, 0, arrow_size * 0.5f);
    glVertex3f(axis_length - arrow_size, 0, -arrow_size * 0.5f);
    glEnd();
    
    // Y axis - Green
    glColor3f(0.2f, 1.0f, 0.2f);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(0, axis_length, 0);
    glEnd();
    // Arrow head
    glBegin(GL_TRIANGLES);
    glVertex3f(0, axis_length, 0);
    glVertex3f(arrow_size * 0.5f, axis_length - arrow_size, 0);
    glVertex3f(-arrow_size * 0.5f, axis_length - arrow_size, 0);
    glVertex3f(0, axis_length, 0);
    glVertex3f(0, axis_length - arrow_size, arrow_size * 0.5f);
    glVertex3f(0, axis_length - arrow_size, -arrow_size * 0.5f);
    glEnd();
    
    // Z axis - Blue
    glColor3f(0.2f, 0.4f, 1.0f);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(0, 0, axis_length);
    glEnd();
    // Arrow head
    glBegin(GL_TRIANGLES);
    glVertex3f(0, 0, axis_length);
    glVertex3f(arrow_size * 0.5f, 0, axis_length - arrow_size);
    glVertex3f(-arrow_size * 0.5f, 0, axis_length - arrow_size);
    glVertex3f(0, 0, axis_length);
    glVertex3f(0, arrow_size * 0.5f, axis_length - arrow_size);
    glVertex3f(0, -arrow_size * 0.5f, axis_length - arrow_size);
    glEnd();
    
    // Restore state
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glPopAttrib();
    
    // Restore full viewport
    glViewport(0, 0, window_width, window_height);
}

// Find a vector perpendicular to the given vector
static Vec3 find_perpendicular(const Vec3& v) {
    // Choose the axis that is least aligned with v
    if (std::abs(v.x) < std::abs(v.y) && std::abs(v.x) < std::abs(v.z)) {
        return v.cross(Vec3::unit_x()).normalized();
    } else if (std::abs(v.y) < std::abs(v.z)) {
        return v.cross(Vec3::unit_y()).normalized();
    } else {
        return v.cross(Vec3::unit_z()).normalized();
    }
}

// Compute parallel transport frames along a spline
// This avoids the singularities of the Frenet frame at inflection points
static std::vector<CurveFrame> compute_parallel_transport_frames(
    const BezierSpline& spline, int samples_per_segment) {

    std::vector<CurveFrame> frames;
    if (spline.empty()) {
        return frames;
    }

    int total_samples = static_cast<int>(spline.segment_count()) * samples_per_segment;
    float dt = static_cast<float>(spline.segment_count()) / static_cast<float>(total_samples);

    // First frame: choose initial normal perpendicular to tangent
    CurveFrame first_frame;
    first_frame.position = spline.evaluate(0.0f);
    first_frame.tangent = spline.tangent(0.0f);
    first_frame.normal = find_perpendicular(first_frame.tangent);
    first_frame.binormal = first_frame.tangent.cross(first_frame.normal).normalized();
    frames.push_back(first_frame);

    // Subsequent frames: parallel transport the frame along the curve
    for (int i = 1; i <= total_samples; ++i) {
        float t = static_cast<float>(i) * dt;
        if (t > static_cast<float>(spline.segment_count())) {
            t = static_cast<float>(spline.segment_count());
        }

        CurveFrame frame;
        frame.position = spline.evaluate(t);
        frame.tangent = spline.tangent(t);

        const CurveFrame& prev = frames.back();

        // Compute rotation from previous tangent to current tangent
        Vec3 axis = prev.tangent.cross(frame.tangent);
        float axis_len = axis.length();

        if (axis_len > 1e-6f) {
            // Normalize the axis
            axis = axis / axis_len;

            // Compute rotation angle
            float cos_angle = prev.tangent.dot(frame.tangent);
            cos_angle = std::max(-1.0f, std::min(1.0f, cos_angle));
            float angle = std::acos(cos_angle);

            // Rodrigues' rotation formula: rotate normal and binormal
            float c = std::cos(angle);
            float s = std::sin(angle);

            // Rotate the normal
            frame.normal = prev.normal * c +
                          axis.cross(prev.normal) * s +
                          axis * (axis.dot(prev.normal)) * (1.0f - c);

            // Rotate the binormal
            frame.binormal = prev.binormal * c +
                            axis.cross(prev.binormal) * s +
                            axis * (axis.dot(prev.binormal)) * (1.0f - c);
        } else {
            // Tangents are parallel (or nearly so), keep previous frame orientation
            frame.normal = prev.normal;
            frame.binormal = prev.binormal;
        }

        // Re-orthonormalize to prevent drift
        frame.normal = frame.normal.normalized();
        frame.binormal = frame.tangent.cross(frame.normal).normalized();
        frame.normal = frame.binormal.cross(frame.tangent).normalized();

        frames.push_back(frame);
    }

    return frames;
}

// Render a tube by extruding a circle along curve frames
static void render_extruded_tube(const std::vector<CurveFrame>& frames,
                                  float compressed_radius, int radial_segments) {
    if (frames.size() < 2) {
        return;
    }

    const float pi = 3.14159265358979f;

    for (size_t i = 0; i + 1 < frames.size(); ++i) {
        const CurveFrame& frame0 = frames[i];
        const CurveFrame& frame1 = frames[i + 1];

        glBegin(GL_QUAD_STRIP);
        for (int j = 0; j <= radial_segments; ++j) {
            float angle = 2.0f * pi * static_cast<float>(j) / static_cast<float>(radial_segments);
            float c = std::cos(angle);
            float s = std::sin(angle);

            // Offset from center of tube
            Vec3 offset0 = frame0.normal * c + frame0.binormal * s;
            Vec3 offset1 = frame1.normal * c + frame1.binormal * s;

            // Vertex positions
            Vec3 pos0 = frame0.position + offset0 * compressed_radius;
            Vec3 pos1 = frame1.position + offset1 * compressed_radius;

            // Normals point outward (same as offset direction)
            glNormal3f(offset0.x, offset0.y, offset0.z);
            glVertex3f(pos0.x, pos0.y, pos0.z);

            glNormal3f(offset1.x, offset1.y, offset1.z);
            glVertex3f(pos1.x, pos1.y, pos1.z);
        }
        glEnd();
    }
}

static void render_graph(const SurfaceGraph& graph, const VisualizerConfig& config) {
    if (!g_show_nodes) return;

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

static void render_spline(const BezierSpline& spline, const VisualizerConfig& config,
                           bool use_tube = true) {
    if (spline.empty()) return;

    if (use_tube && config.render_as_tube) {
        // Render as extruded tube
        auto frames = compute_parallel_transport_frames(spline, config.spline_samples);
        glEnable(GL_LIGHTING);
        render_extruded_tube(frames, config.yarn_compressed_radius, config.tube_radial_segments);
    } else {
        // Legacy line rendering
        glDisable(GL_LIGHTING);
        auto polyline = spline.to_polyline_fixed(config.spline_samples);
        if (polyline.size() < 2) return;

        glBegin(GL_LINE_STRIP);
        for (const auto& pt : polyline) {
            glVertex3f(pt.x, pt.y, pt.z);
        }
        glEnd();
    }
}

static void render_geometry(const GeometryPath& geometry, const VisualizerConfig& config) {
    if (!g_show_geometry || !config.show_geometry) return;

    // Set up for rendering (line width for fallback mode)
    glLineWidth(config.spline_line_width);

    // Check if we're viewing geometry history
    if (g_viewing_geometry_history && g_current_geometry_snapshot >= 0 &&
        g_current_geometry_snapshot < static_cast<int>(g_geometry_snapshots.size())) {
        // Show the accumulated spline at the current snapshot
        const auto& snap = g_geometry_snapshots[g_current_geometry_snapshot];
        glColor3f(0.9f, 0.8f, 0.2f);  // Yellow/gold for yarn
        render_spline(snap.spline, config, true);

        // Also highlight the most recently added segment in a different color
        if (!snap.spline.empty()) {
            glColor3f(1.0f, 0.2f, 0.2f);  // Red for the latest curve
            glLineWidth(config.spline_line_width + 2.0f);
            const auto& segments = snap.spline.segments();
            if (!segments.empty()) {
                BezierSpline last_segment;
                last_segment.add_segment(segments.back());
                // Render latest segment with slightly larger compressed_radius for visibility
                VisualizerConfig highlight_config = config;
                highlight_config.yarn_compressed_radius = config.yarn_compressed_radius * 1.1f;
                render_spline(last_segment, highlight_config, true);

                // Draw a green sphere at the endpoint of the spline
                glEnable(GL_LIGHTING);
                glColor3f(0.2f, 1.0f, 0.2f);  // Bright green for endpoint
                Vec3 endpoint = segments.back().end();
                draw_sphere(endpoint.x, endpoint.y, endpoint.z, config.node_size * 1.5f);

                // Draw a cyan sphere at the start point of the latest segment
                glColor3f(0.2f, 1.0f, 1.0f);  // Cyan for start of latest segment
                Vec3 startpoint = segments.back().start();
                draw_sphere(startpoint.x, startpoint.y, startpoint.z, config.node_size * 1.2f);
            }
        }
        return;
    }

    // Show full geometry
    if (geometry.segments().empty()) return;

    // Build a combined spline from all segments for continuous rendering
    BezierSpline combined_spline;
    for (const auto& seg : geometry.segments()) {
        for (const auto& bez : seg.curve.segments()) {
            combined_spline.add_segment(bez);
        }
    }

    if (combined_spline.empty()) return;

    // Draw the yarn path as extruded tube or line
    glColor3f(0.9f, 0.8f, 0.2f);  // Yellow/gold for yarn
    render_spline(combined_spline, config, true);
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
    const Gauge& gauge,
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

    // Do an initial clear
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
    g_show_nodes = true;
    g_show_geometry = false;  // No geometry in surface-only mode

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

    // Thread synchronization
    std::mutex snapshot_mutex;
    std::atomic<bool> solver_running(false);
    std::atomic<bool> should_stop(false);
    int current_iteration = 0;
    float current_energy = energy;

    // Take initial snapshot
    g_snapshots.push_back(take_snapshot(graph, iteration, energy));

    log->info("Visualizer started. Controls:");
    log->info("  mouse drag=rotate, scroll=zoom");
    log->info("  space=pause/resume, q=quit, r=reset camera");
    log->info("  n=toggle nodes, g=toggle geometry");
    log->info("  left/right=frame by frame, pgup/pgdn=30 frames");
    log->info("  home=first frame, end=live view");

    // Start solver thread
    solver_running = true;
    std::thread solver_thread([&]() {
        auto log = yarnpath::logging::get_logger();
        
        StepCallback step_callback = [&](const SurfaceGraph& g, int iter, float curr_energy, float energy_change) -> bool {
            {
                std::lock_guard<std::mutex> lock(snapshot_mutex);
                current_iteration = iter;
                current_energy = curr_energy;
                
                // Take snapshot at configured interval
                if (iter % viz_config.snapshot_interval == 0) {
                    if (static_cast<int>(g_snapshots.size()) < viz_config.max_snapshots) {
                        g_snapshots.push_back(take_snapshot(g, iter, curr_energy));
                    }
                }
            }
            
            // Log progress periodically (already done by solver, but keep for thread-specific logging)
            if (iter % 100 == 0) {
                log->debug("Solver iteration {}: energy={:.6f}, delta={:.6f}", iter, curr_energy, energy_change);
            }
            
            // Return false only if explicitly requested to stop
            return !should_stop;
        };
        
        SolveResult solver_result = SurfaceSolver::solve(graph, yarn, gauge, solve_config, step_callback);
        solver_running = false;
        
        log->info("Solver finished: converged={}, iterations={}, energy={}",
                  solver_result.converged, solver_result.iterations, solver_result.final_energy);
    });

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

        // Update current state from solver thread
        if (solver_running) {
            {
                std::lock_guard<std::mutex> lock(snapshot_mutex);
                iteration = current_iteration;
                energy = current_energy;
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
        // When solver is running and not viewing history, just render the graph's
        // current state directly without applying snapshots (avoids race condition)

        // Render
        render_graph(graph, viz_config);

        // Render XYZ orientation gizmo
        render_axis_gizmo(width, height, g_camera);

        // Swap buffers and poll events
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // Stop solver thread if still running
    should_stop = true;
    if (solver_thread.joinable()) {
        solver_thread.join();
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

VisualizerResult visualize_with_geometry(
    SurfaceGraph& graph,
    const YarnPath& yarn_path,
    const YarnProperties& yarn,
    const Gauge& gauge,
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

    // Do an initial clear
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glfwSwapBuffers(window);
    glfwPollEvents();

    // Set up callbacks
    glfwSetMouseButtonCallback(window, mouse_button_callback);
    glfwSetCursorPosCallback(window, cursor_position_callback);
    glfwSetScrollCallback(window, scroll_callback);
    glfwSetKeyCallback(window, key_callback);

    // Create a mutable config with yarn compressed_radius from properties
    VisualizerConfig config = viz_config;
    config.yarn_compressed_radius = yarn.compressed_radius;

    // Initialize camera
    g_camera.distance = config.camera_distance;
    g_rotation_speed = config.rotation_speed;
    g_zoom_speed = config.zoom_speed;
    g_paused = !config.auto_run;
    g_show_nodes = true;
    g_show_geometry = config.show_geometry;

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
    bool geometry_built = false;
    GeometryPath geometry;

    // Take initial snapshot
    g_snapshots.push_back(take_snapshot(graph, iteration, energy));

    log->info("Visualizer started (with geometry). Controls:");
    log->info("  mouse drag=rotate, scroll=zoom");
    log->info("  space=pause/resume, q=quit, r=reset camera");
    log->info("  n=toggle nodes, g=toggle geometry");
    log->info("  left/right=frame by frame, pgup/pgdn=30 frames");
    log->info("  home=first frame, end=live view");
    log->info("  b/f=step geometry build, 0=show full geometry");

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

        // Run geometry build when ready (this function doesn't have continuous solving)
        if (!g_paused && !g_viewing_history && !geometry_built) {
            log->info("Building geometry from relaxed surface...");

            // Clear any previous geometry snapshots
            g_geometry_snapshots.clear();
            g_current_geometry_snapshot = -1;
            g_viewing_geometry_history = false;

            // Build geometry with callback to capture snapshots
            geometry = build_geometry_with_callback(yarn_path, graph, yarn, gauge,
                [&log](SegmentId seg_id, const std::string& desc, const BezierSpline& spline) {
                    GeometrySnapshot snap;
                    snap.segment_id = seg_id;
                    snap.description = desc;
                    snap.spline = spline;
                    g_geometry_snapshots.push_back(snap);
                    log->debug("  geometry step: seg {} - {}", seg_id, desc);
                });

            geometry_built = true;
            log->info("Geometry built with {} segments, {} curve steps recorded",
                     geometry.segments().size(), g_geometry_snapshots.size());
            log->info("  Press B/F to step through geometry build, 0 to show full geometry");
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

        // Log geometry step when it changes
        if (g_viewing_geometry_history && g_current_geometry_snapshot >= 0 &&
            g_current_geometry_snapshot < static_cast<int>(g_geometry_snapshots.size())) {
            if (g_current_geometry_snapshot != g_last_logged_geometry_snapshot) {
                const auto& gsnap = g_geometry_snapshots[g_current_geometry_snapshot];
                Vec3 endpoint = Vec3::zero();
                if (!gsnap.spline.empty()) {
                    endpoint = gsnap.spline.segments().back().end();
                }
                log->info("Geometry step {}/{}: seg {} - {} | endpoint: ({:.2f}, {:.2f}, {:.2f})",
                         g_current_geometry_snapshot + 1, g_geometry_snapshots.size(),
                         gsnap.segment_id, gsnap.description,
                         endpoint.x, endpoint.y, endpoint.z);
                g_last_logged_geometry_snapshot = g_current_geometry_snapshot;
            }
        } else if (!g_viewing_geometry_history && g_last_logged_geometry_snapshot != -1) {
            log->info("Showing full geometry");
            g_last_logged_geometry_snapshot = -1;
        }

        // Render
        render_graph(graph, config);
        if (geometry_built) {
            render_geometry(geometry, config);
        }

        // Render XYZ orientation gizmo
        render_axis_gizmo(width, height, g_camera);

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
    g_geometry_snapshots.clear();
    g_current_geometry_snapshot = -1;
    g_viewing_geometry_history = false;
    g_last_logged_geometry_snapshot = -2;

    glfwDestroyWindow(window);
    glfwTerminate();

    log->info("Visualization ended. {} snapshots recorded, {} geometry steps.",
              frame_count + 1, g_geometry_snapshots.size());

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
    const Gauge&,
    const SolveConfig&,
    const VisualizerConfig&) {

    auto log = yarnpath::logging::get_logger();
    log->error("Visualization not available - compile with GLFW and OpenGL");
    return VisualizerResult{};
}

VisualizerResult visualize_with_geometry(
    SurfaceGraph&,
    const YarnPath&,
    const YarnProperties&,
    const Gauge&,
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
