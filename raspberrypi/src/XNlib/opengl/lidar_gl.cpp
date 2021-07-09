// clang-format off
/*
    compile: cmake --build .

    usage:
        Run with networking:    
            ./lidar
        
        View saved pointcloud:
            ./lidar filename.pc
*/
// clang-format on

#define PIO_VIRTUAL

#define GLM_ENABLE_EXPERIMENTAL
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include "xn_robot_host_app.hpp"
#include "xn_gui.hpp"

#include <pthread.h>
#include <numeric>

#include <fstream>
#include <iostream>
#include <filesystem>

#include <fstream>
#include <signal.h>
#include <glm/gtx/rotate_vector.hpp>

#define SCR_WIDTH 1200
#define SCR_HEIGHT 800
#define PORT 4000

using namespace xn;
namespace fs = std::filesystem;

struct BoundedPoints {
    OBB box;
    std::vector<glm::vec3> points;
};
std::vector<BoundedPoints> boxes;

bool run = true;
const glm::vec2 win_size_pixels(SCR_WIDTH, SCR_HEIGHT);

// input
gl::Camera cam;
glm::dvec2 cursor(0);
float orth_zoom = 6;
int max_tilt_error = 8;

// timing
float deltaTime = 0.0f; // time between current frame and last frame
float lastFrame = 0.0f;

std::vector<PointCloud> PointCloud::all;
DrawNode DrawNode::scene;
int DrawNode::id_count = 0;

gl::Texture2D cam_tex;

int pi_sock;
char *pic_buffer;
int32_t pic_buflen = 0;
bool pic_needs_update = false;

bool drawBoxes = false, orth = true, conn_accepted = false, orbit = true;
void processInput(gl::Window &window);
void mouse_button_callback(GLFWwindow *window, int button, int action, int mods);

void int_handler(int sig) {
    SIG_IGN(sig);
    run = 0;
    if (!conn_accepted)
        exit(0);
    close(esp_sock);
}

void *robot_io_thread(void *argv) {
    while (true) {
        lidar_scan(esp_sock, car_position, car_angle);

        float t = 0;
        while (path.size() < 1) {
            time_sleep(0.5);
            t += 0.5;
            if (t >= 1.0f) {
                close(esp_sock);
                esp_sock = accept_connection_blocking(4000);
                if (settings.should_scan) {
                    lidar_scan(esp_sock, car_position, car_angle);
                    settings.should_scan = false;
                }

                if (settings.should_restart_esp) {
                    RestartEspInstruction r;
                    send_pack(esp_sock, r);
                    settings.should_restart_esp = false;
                }
                t = 0;
            }
        }

        car_follow_path(esp_sock, path);
        path.clear();
    }
    return NULL;
}

void *cam_io_thread(void *argv) {
    while (true) {
        read_int<int32_t>(pi_sock, pic_buflen);
        delete pic_buffer;
        pic_buffer = new char[pic_buflen];
        read_buf(pi_sock, pic_buffer, pic_buflen, 5);
        pic_needs_update = true;
    }
}

void *cam_servo_ctl_thread(void *argv) {
    int x_prev = 0, y_prev = 0;
    while (true) {
        if (x_prev != settings.cam_servo_width_x) {
            int32_t p = 23, v = settings.cam_servo_width_x;
            send_int(pi_sock, p);
            send_int(pi_sock, v);
        }

        if (y_prev != settings.cam_servo_width_y) {
            int32_t p = 24, v = settings.cam_servo_width_y;
            send_int(pi_sock, p);
            send_int(pi_sock, v);
        }

        x_prev = settings.cam_servo_width_x;
        y_prev = settings.cam_servo_width_y;
        time_sleep(1.0 / 60);
    }
}

int main(int argc, char *argv[]) {
    pthread_t tid_car, tid_cam, tid_cam_servo;
    std::vector<std::string> pc_filepaths;
    scan_drawnode = DrawNode::scene.addChild(new DrawNode());

    if (argc > 1 && strcmp(argv[1], "-i") != 0) {
        // read savefile
        PointCloud::read_pointclouds(argv[1], PointCloud::all);

        std::string pc_path = "pointcloud";
        for (const auto &e : fs::directory_iterator(pc_path)) {
            std::string s = e.path();
            s.erase(0, strlen("pointcloud/"));
            pc_filepaths.push_back(s);
        }

    } else {
        // connect to lidar controller
        signal(SIGINT, int_handler);

        // connect raspi
        pi_sock = accept_connection_blocking(5000);
        if (pi_sock < 0)
            error("ERROR on accept");
        pthread_create(&tid_cam, NULL, &cam_io_thread, NULL);              // camera stream
        pthread_create(&tid_cam_servo, NULL, &cam_servo_ctl_thread, NULL); // camera servo ctl

        // connect esp
        esp_sock = accept_connection_blocking(PORT);
        if (esp_sock < 0)
            error("ERROR on accept");

        conn_accepted = true;
        if (pthread_mutex_init(&mut, NULL) != 0) {
            printf("\n mutex init has failed\n");
            return 1;
        }

        pthread_create(&tid_car, NULL, &robot_io_thread, NULL); // read lidar values
    }

    // convert pointcloud to pathfinding graph
    if (PointCloud::all.size() > 0) {
        PointCloud::march_squares(PointCloud::all, 50, 50, gridbox_size, car_position, gridgraph,
                                  grid_offset);
        graph = preprocess_graph(gridgraph);
    }

    // init opengl
    gl::Window window(SCR_WIDTH, SCR_HEIGHT, "boxin");
    glfwSetMouseButtonCallback(window.handle, mouse_button_callback);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Setup Dear ImGui context
    gui::init(window.handle);

    color_shader = Shader("shaders/camera.vs", "shaders/camera.fs");
    Shader tex_shader("shaders/textures.vs", "shaders/textures.fs");
    Shader shader_2d("shaders/2d.vs", "shaders/2d.fs");

    gl::Texture2D test_tex("xabnab.png", GL_TEXTURE0, false);

    // init draw buffers
    gl::VertexArrayInfo box, ax;
    gl::gen_arrays(gl::cube_verts, sizeof(gl::cube_verts), box, 5);
    gl::gen_arrays(gl::line, sizeof(gl::line), ax);
    boxmesh.init();

    load_scene();

    DUMP(glfwGetVersionString());
    while (!window.shouldClose() && run) {
        glm::vec3 def_scale{0.05, 0.05, 0.05};
        glm::vec3 car_scale = def_scale * 3.0f;
        glm::vec3 car_pos_draw(car_position.x, 0, car_position.y);

        float currentFrame = glfwGetTime();
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        glfwPollEvents();
        if (!settings.ignore_keystrokes) {
            processInput(window);
        }

        gui::update_frame();
        gui::update_menu_bar();
        gui::update_window_settings();

        if (conn_accepted) {
            if (pic_needs_update) {
                cam_tex = gl::Texture2D((unsigned char *)pic_buffer, pic_buflen, GL_TEXTURE0, true);
                pic_needs_update = false;
            }

            struct CustomConstraints {
                // Helper functions to demonstrate programmatic constraints
                static void tex_ratio(ImGuiSizeCallbackData *data) {
                    float ratio = (float)cam_tex.width / cam_tex.height;
                    data->DesiredSize.x = (data->DesiredSize.y - 50) * ratio;
                }
            };
            ImGui::SetNextWindowSizeConstraints(ImVec2(0, 0), ImVec2(FLT_MAX, FLT_MAX),
                                                CustomConstraints::tex_ratio);
            ImGui::Begin("Camera");
            ImVec2 vMin = ImGui::GetWindowContentRegionMin();
            ImVec2 vMax = ImGui::GetWindowContentRegionMax();
            ImVec2 winsize(vMax.x - vMin.x, vMax.y - vMin.y - 50);
            ImGui::Image((ImTextureID)cam_tex.id, winsize);

            ImGui::SliderInt("Pan", &settings.cam_servo_width_x, 500, 2500);
            ImGui::SliderInt("Tilt", &settings.cam_servo_width_y, 500, 2500);
            ImGui::End();
        }

        // move camera
        if (!orth) {
            cam.projection = glm::perspective(glm::radians(cam.fov),
                                              (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);
            cam.view = glm::lookAt(cam.pos, cam.target, cam.up);
            cam.front = glm::normalize(cam.target - cam.pos);
        } else {
            float aspect = (float)SCR_WIDTH / (float)SCR_HEIGHT;
            cam.projection = glm::ortho(-orth_zoom * aspect, orth_zoom * aspect, -orth_zoom,
                                        orth_zoom, cam.rad, 10000.0f);
            cam.view = glm::lookAt(glm::vec3(0, 6, 0), glm::vec3(0, 0, 0), glm::vec3(0, 0, -1));
        }

        gl::cls(0.2f, 0.3f, 0.3f);

        // draw textured dome
        // begin_3d(cam, tex_shader);
        // tex_shader.setTransform(glm::vec3(0), glm::vec3(30));
        // dome_surface.draw();

        begin_3d(cam, color_shader);
        car_drawnode->setModelRecursive();
        car_drawnode->transform(car_pos_draw, car_scale, -M_PI_2 + car_angle);

        grid_drawnode->setModelRecursive();
        grid_drawnode->transform(grid_offset -
                                 glm::vec3(gridbox_size * 0.5, 0, gridbox_size * 0.5));

        // draw pca obbs
        if (drawBoxes) {
            color_shader.setVec4("color", glm::vec4(1.0f, 1.0f, 0.0f, 1.0f));
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            for (BoundedPoints &bp : boxes)
                gl::draw_OBB(bp.box, color_shader);
        }

        pthread_mutex_lock(&mut);
        gl::draw_path(box.VAO, color_shader, gridgraph, grid_offset, gridbox_size, path);
        pthread_mutex_unlock(&mut);

        // glm::vec3 grid_outline_offset(grid_offset);
        // grid_outline_offset.x = fmod(grid_offset.x, 1);
        // grid_outline_offset.z = fmod(grid_offset.z, 1);
        // gl::draw_grid(grid.VAO, color_shader, gridbox_size, 30, 30, grid_outline_offset);

        // gl::draw_axes(ax.VAO, color_shader);

        // add new navmeshes to scene
        if (navmesh_drawnode->children.size() < nav_verts.size()) {
            for (unsigned i = navmesh_drawnode->children.size(); i < nav_verts.size(); i++) {
                DrawNode *n =
                    navmesh_drawnode->addChild(new DrawNode(&color_shader, new Mesh(nav_verts[i])));
                n->addUniform("color", glm::vec4(0.8f, 0.6f, 0.4f, 0.5f));
            }
        }

        if (settings.graph_needs_update) {
            update_graph_boxes();
            settings.graph_needs_update = false;
        }

        if (settings.scan_needs_update) {
            scan_drawnode->children.clear();
            for (auto &p : PointCloud::all) {
                DrawNode *c = scan_drawnode->addChild(
                    new DrawNode(&color_shader, Mesh::gen_box_batch(p.points)));
                c->addUniform("color", gl::sample_colors[0]);
            }
            settings.scan_needs_update = false;
        }

        DrawNode::scene.draw();
        gui::draw();

        window.flip();

        double remaining_time = 1.0 / 60 - (glfwGetTime() - currentFrame);
        if (remaining_time > 0)
            time_sleep(remaining_time);
    }

    close(esp_sock);
    // scene.cleanupRecursive();
    gui::cleanup();
    glfwTerminate();
    return 0;
}

void processInput(gl::Window &window) {
    // glfwSetCursorPos(window, 0, 0);
    // glfwGetCursorPos(window, &cursor.x, &cursor.y);
    cursor = window.getCursorPos();
    // printf("cursor: (%.3f, %.3f)\n", cursor.x, cursor.y);

    float cameraSpeed = 2.5 * deltaTime;
    static bool e_pressed = false, c_pressed = false, f_pressed = false;

    auto handle_keypress = [&window](int key, float &accum, float incr) {
        if (window.getKey(key) == GLFW_PRESS)
            accum += incr;
    };
    auto handle_toggle = [&window](int key, bool &button_pressed, bool &toggle_var) {
        if (window.getKey(key) == GLFW_PRESS && !button_pressed) {
            button_pressed = true;
            toggle_var = !toggle_var;
        }
        if (window.getKey(key) == GLFW_RELEASE && button_pressed) {
            button_pressed = false;
        }
    };

    // camera movement
    switch (cam.type) {
    case gl::CAMERA_ORBIT:
        handle_keypress(GLFW_KEY_W, cam.rad, -cameraSpeed);
        handle_keypress(GLFW_KEY_S, cam.rad, cameraSpeed);
        handle_keypress(GLFW_KEY_A, cam.ang, cameraSpeed);
        handle_keypress(GLFW_KEY_D, cam.ang, -cameraSpeed);
        cam.pos = glm::vec3(cosf(cam.ang) * cam.rad, cam.height, sinf(cam.ang) * cam.rad);
        break;
    case gl::CAMERA_FLY: {
        glm::vec3 r(0);
        handle_keypress(GLFW_KEY_UP, r.z, cameraSpeed);
        handle_keypress(GLFW_KEY_DOWN, r.z, -cameraSpeed);
        handle_keypress(GLFW_KEY_LEFT, r.x, -cameraSpeed);
        handle_keypress(GLFW_KEY_RIGHT, r.x, cameraSpeed);

        glm::vec3 right = glm::cross(cam.front, gl::axes.y);
        glm::vec3 up = glm::cross(cam.front, right);
        glm::vec3 d = glm::normalize(cam.target - cam.pos);
        d = glm::rotate(d, r.x, up);
        d = glm::rotate(d, r.z, right);
        cam.target = cam.pos + d;

        glm::vec3 t(0);
        handle_keypress(GLFW_KEY_W, t.z, cameraSpeed);
        handle_keypress(GLFW_KEY_S, t.z, -cameraSpeed);
        handle_keypress(GLFW_KEY_A, t.x, -cameraSpeed);
        handle_keypress(GLFW_KEY_D, t.x, cameraSpeed);
        cam.pos += cam.front * t.z;
        cam.pos += right * t.x;

        cam.target += cam.front * t.z;
        cam.target += right * t.x;
        break;
    }
    }

    // toggle bounding boxes
    handle_toggle(GLFW_KEY_E, e_pressed, drawBoxes);

    // toggle top-down orthographic view
    bool c_pressed_prev = c_pressed;
    handle_toggle(GLFW_KEY_C, c_pressed, orth);
    if (c_pressed_prev != c_pressed)
        cam.rad = 5;

    handle_toggle(GLFW_KEY_F, f_pressed, orbit);
    if (orbit) {
        cam.type = gl::CAMERA_ORBIT;
        cam.target = glm::vec3(0);
    } else
        cam.type = gl::CAMERA_FLY;

    if (window.getKey(GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS &&
        window.getKey(GLFW_KEY_S) == GLFW_PRESS) {
        gui::MenuBarState.show_popup_save = true;
    }

    if (window.getKey(GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        window.close();
        run = 0;
    }
}

void mouse_button_callback(GLFWwindow *window, int button, int action, int mods) {
    if (!settings.ignore_keystrokes && !settings.window_settings_hovered) {
        if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
            glfwGetCursorPos(window, &cursor.x, &cursor.y);
            printf("cursor: (%.3f, %.3f)\n", cursor.x, cursor.y);

            if (graph.size() > 0) {
                glm::vec2 target;
                glm::vec3 start_world(car_position.x, 0, car_position.y);
                glm::vec2 start = world_to_grid(start_world, gridbox_size, grid_offset);
                Plane p{0, gl::axes.y};
                glm::vec3 ray_a, ray_b, q;
                cam.get_pixelray(cursor, win_size_pixels, ray_a, ray_b);
                float t = 0;
                if (collision_segment_plane(ray_a, ray_b, p, t, q)) {
                    target_world = q;
                    target = world_to_grid(q, gridbox_size, grid_offset);
                }
                target.x = clamp<float>(target.x, 0, gridgraph.size());
                target.y = clamp<float>(target.y, 0, gridgraph.front().size());
                start.x = clamp<float>(start.x, 0, gridgraph.size());
                start.y = clamp<float>(start.y, 0, gridgraph.front().size());
                path = A_star(graph, start.x, start.y, target.x, target.y);
            }
        }
    }
}