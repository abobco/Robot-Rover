/*
        compile: cmake --build .

        usage:
                Run with networking:
                        ./lidar

                View saved pointcloud:
                        ./lidar filename.pc

  current sensor math
        50 mv / x = 125 / 1
        x / 0.61 = 125 / 1
*/

#define PIO_VIRTUAL
#define HAVE_STRUCT_TIMESPEC
#define GLM_ENABLE_EXPERIMENTAL

#define XN_LIT true

#include <opencv2/opencv.hpp>

#include "app/app_threads.hpp"
#include "opengl/xn_gui.hpp"
#include "robo/xn_yolo.hpp"
#include "xn_robot_host_app.hpp"

#include "robo/xn_ssh.hpp"
#include "util/load_scene_json.hpp"
#include <fstream>
#include <sstream>

using namespace xn;

std::string settings_file_path = "../config/xn_settings.json";

// input
gl::Camera cam;
glm::dvec2 cursor(0);
float orth_zoom = 6;

// int raspipe[2];
// std::stringstream consolestream;
// std::stringstream console_out_stream;

// gyro tilt threshold to trigger recovery procedure
int max_tilt_error = 8;

// frame timing
float deltaTime = 0.0f;
float lastFrame = 0.0f;

// static scene objects
DrawNode DrawNode::scene;
std::vector<PointCloud> PointCloud::all;
int DrawNode::id_count = 0;
std::unordered_map<std::string, Shader> DrawNode::shader_map;
std::unordered_map<std::string, DrawNode *> DrawNode::node_map;

std::vector<BoundedPoints> boxes;
json jset;

bool orth = true, orbit = true, mouse_held = false;

void processInput(gl::Window &window);
void framebuffer_size_callback(GLFWwindow *window, int width, int height);
void mouseclick_callback(GLFWwindow *window, int button, int action, int mods);
void window_pos_callback(GLFWwindow *window, int xpos, int ypos);
void update_path(GLFWwindow *window);

int main(int argc, char *argv[]) {
  signal(SIGINT, int_handler);
  std::vector<std::thread> threads;
  std::vector<std::string> pc_filepaths;
  jset = load_settings_json(settings_file_path);
  draw_map["scan"] = DrawNode::scene.addChild(new DrawNode());

  xn_printf("%d threads supported by hardware\n",
            std::thread::hardware_concurrency());

  ssh_session ssh;
  SshConsole console(jset, &ssh);

  threads.push_back(std::thread(ik_sim_thread));

  if (argc > 1 && strcmp(argv[1], "-i") != 0) {
    // read savefile
    std::cout << argv[1];
    PointCloud::read_pointclouds(argv[1], PointCloud::all);

    // threads.push_back(std::thread(car_sim_thread));
  }
  threads.push_back(std::thread(yolo_thread, jset));

  // convert pointcloud to pathfinding graph
  if (PointCloud::all.size() > 0) {
    PointCloud::march_squares(
        PointCloud::all, 50, 50, AppState::get().gridbox_size,
        AppState::get().car_position, AppState::get().gridgraph,
        AppState::get().grid_offset);
    AppState::get().graph = preprocess_graph(AppState::get().gridgraph);
  }

  // init opengl
  gl::Window window(jset["resolution"][0], jset["resolution"][1], "boxin");
  if (!jset["window_position"].is_null())
    glfwSetWindowPos(window.handle, jset["window_position"][0],
                     jset["window_position"][1]);

  window.resize(jset["resolution"][0], jset["resolution"][1]);
  glfwSetMouseButtonCallback(window.handle, mouseclick_callback);
  glfwSetFramebufferSizeCallback(window.handle, framebuffer_size_callback);
  glfwSetWindowPosCallback(window.handle, window_pos_callback);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glfwSwapInterval(1);

  // Setup Dear ImGui context
  std::streambuf *old = std::cout.rdbuf(log_buffer.rdbuf());
  gui::init(window.handle);

  color_shader = Shader("../shaders/camera.vs", "../shaders/camera.fs");
  lit_shader = Shader("../shaders/lit.vs", "../shaders/lit.fs");
  Shader tex_shader("../shaders/textures.vs", "../shaders/textures.fs");
  Shader shader_2d("../shaders/2d.vs", "../shaders/2d.fs");

  // init draw buffers
  gl::VertexArrayInfo box;
  gl::gen_arrays(gl::cube_verts, sizeof(gl::cube_verts), box, 5);
  boxmesh.init();

  // load_scene_json("../config/scene.json");
  // exit(0);

  // init render tree
  try {

    load_scene();

    DUMP(glfwGetVersionString());
    while (!window.shouldClose() && AppState::get().run) {
      glm::vec3 def_scale{0.05, 0.05, 0.05};
      glm::vec3 car_scale = def_scale * 3.0f;

      float currentFrame = (float)glfwGetTime();
      deltaTime = currentFrame - lastFrame;
      lastFrame = currentFrame;

      glfwPollEvents();
      if (!AppState::get().ignore_keystrokes &&
          !ImGui::GetIO().WantCaptureMouse) {
        processInput(window);
      }

      gui::update_frame();
      gui::update_menu_bar();
      gui::update_window_settings();

      if (AppState::get().conn_accepted) {
        if (AppState::get().pic_needs_update) {
          cam_tex = gl::Texture2D(&AppState::get().ocv_buf.front(),
                                  (int)AppState::get().ocv_buf.size(),
                                  GL_TEXTURE0, false);
          AppState::get().pic_needs_update = false;
        }
        gui::update_cam_window(cam_tex);
      }

      // move camera
      float aspect = (float)gl::win_size_pixels.x / gl::win_size_pixels.y;
      if (!orth) {
        cam.projection =
            glm::perspective(glm::radians(cam.fov), aspect, 0.1f, 100.0f);
        cam.view = glm::lookAt(cam.pos, cam.target, cam.up);
        cam.front = glm::normalize(cam.target - cam.pos);
      } else {
        cam.projection =
            glm::ortho(-orth_zoom * aspect, orth_zoom * aspect, -orth_zoom,
                       orth_zoom, cam.rad * 0.5f, 10000.0f);
        cam.view = glm::lookAt(glm::vec3(0, 6, 0), glm::vec3(0, 0, 0),
                               glm::vec3(0, 0, -1));
      }

      if (mouse_held) {
        update_path(window.handle);
      }

      scene_update(cam, boxes);

      AppState::get().mut.lock();
      DrawNode::scene.draw();
      AppState::get().mut.unlock();

      // update log windows
      {
        static unsigned prev_len = 0;
        std::string content = log_buffer.str();
        if (content.size() > prev_len) {
          log_local.Clear();
          log_local.AddLog(content.c_str());
          prev_len = (unsigned)content.size();
        }

        if (AppState::get().gui_windows_need_update) {
          ImGui::SetNextWindowPos(
              ImVec2(0, ImGui::GetMainViewport()->GetCenter().y * 2 - 200), 0,
              ImVec2(0, 0));
        }
        ImGui::SetNextWindowBgAlpha(0.7f);
        ImGui::Begin("stdout");
        ImGuiTabBarFlags tab_bar_flags = ImGuiTabBarFlags_None;
        if (ImGui::BeginTabBar("MyTabBar", tab_bar_flags)) {
          if (ImGui::BeginTabItem("local")) {
            ImGui::BeginChild("content");
            log_local.DrawTextWrapped();
            ImGui::EndChild();
            ImGui::EndTabItem();
          }
          if (ImGui::BeginTabItem("esp")) {
            ImGui::BeginChild("content");
            log_esp.DrawTextWrapped();
            ImGui::EndChild();
            ImGui::EndTabItem();
          }
          ImGui::EndTabBar();
        }
        ImGui::End();
      }
      {
        bool t = true;
        if (AppState::get().console->istream.tellp() > 0) {
          std::string contents;
          contents = AppState::get().console->istream.str();
          console.AddLog("%s", contents.c_str());

          AppState::get().console->istream.str(std::string());
        }
        console.Draw("Ssh: Console", &t);
      }
      gui::draw();
      AppState::get().gui_windows_need_update = false;

      window.flip();
    }
  } catch (const std::exception &exc) {
    std::cerr << exc.what();
  }

  closesocket(AppState::get().console->sockets.esp_data);
  gui::cleanup();
  glfwTerminate();
  AppState::get().run = false;
  for (auto &t : threads)
    t.join();
  return 0;
}

void window_pos_callback(GLFWwindow *window, int xpos, int ypos) {
  jset["window_position"][0] = xpos;
  jset["window_position"][1] = ypos;
  std::fstream f(settings_file_path);
  f << std::setw(4) << jset << std::endl;
  f.close();
}

void framebuffer_size_callback(GLFWwindow *window, int width, int height) {
  glViewport(0, 0, width, height);
  gl::win_size_pixels = glm::ivec2(width, height);
  AppState::get().gui_windows_need_update = true;

  jset["resolution"][0] = width;
  jset["resolution"][1] = height;
  std::fstream f(settings_file_path);
  f << std::setw(4) << jset << std::endl;
  f.close();
}

void processInput(gl::Window &window) {
  cursor = window.getCursorPos();

  float cameraSpeed = 2.5f * deltaTime;
  static bool e_pressed = false, c_pressed = false, f_pressed = false;

  auto handle_keypress = [&window](int key, float &accum, float incr) {
    if (window.getKey(key) == GLFW_PRESS)
      accum += incr;
  };
  auto handle_toggle = [&window](int key, bool &button_pressed,
                                 bool &toggle_var) {
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
    cam.pos =
        glm::vec3(cosf(cam.ang) * cam.rad, cam.height, sinf(cam.ang) * cam.rad);
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
  handle_toggle(GLFW_KEY_E, e_pressed, AppState::get().should_draw_boxes);

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
    AppState::get().run = 0;
  }
}

void mouseclick_callback(GLFWwindow *window, int button, int action, int mods) {
  if (!AppState::get().ignore_keystrokes &&
      !AppState::get().window_settings_hovered) {
    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
      mouse_held = true;
    } else if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE) {
      mouse_held = false;
    }
  }
}

void update_path(GLFWwindow *window) {
  glfwGetCursorPos(window, &cursor.x, &cursor.y);
  // xn_printf("cursor: (%.3f, %.3f)\n", cursor.x, cursor.y);

  if (AppState::get().graph.size() > 0 && AppState::get().path_mut.try_lock()) {
    glm::ivec2 target;
    glm::vec3 start_world(AppState::get().car_position.x, 0,
                          AppState::get().car_position.y);
    glm::ivec2 start = world_to_grid(start_world, AppState::get().gridbox_size,
                                     AppState::get().grid_offset);
    Plane p{0, gl::axes.y};
    glm::vec3 ray_a, ray_b, q;
    cam.get_pixelray(cursor, gl::win_size_pixels, ray_a, ray_b);
    float t = 0;
    if (collision_segment_plane(ray_a, ray_b, p, t, q)) {
      AppState::get().target_world = q;
      target = world_to_grid(q, AppState::get().gridbox_size,
                             AppState::get().grid_offset);
    }
    target.x = clamp(target.x, 0, (int)AppState::get().gridgraph.size());
    target.y =
        clamp(target.y, 0, (int)AppState::get().gridgraph.front().size());
    start.x = clamp(start.x, 0, (int)AppState::get().gridgraph.size());
    start.y = clamp(start.y, 0, (int)AppState::get().gridgraph.front().size());
    AppState::get().path =
        A_star(AppState::get().graph, start.x, start.y, target.x, target.y);
    AppState::get().path_mut.unlock();
  }
}
