#pragma once
#include "../robo/xn_log.hpp"
// #include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include "../robo/xn_console.hpp"
#include "../robo/xn_pointcloud.hpp"
#include "../xn_robot_host_app.hpp"
#include "xn_renderer.hpp"

// #include "xn_gl.hpp"

namespace xn {

// struct {
//     int scan_range.x = 200;
//     int scan_range.y = 800;
// } settings;

namespace gui {

void init(GLFWwindow *winhandle, const char *glsl_version = "#version 130") {
  // Setup Dear ImGui context
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO &io = ImGui::GetIO();
  (void)io;

  // Setup Dear ImGui style
  ImGui::StyleColorsDark();

  // Setup Platform/Renderer backends
  ImGui_ImplGlfw_InitForOpenGL(winhandle, true);
  ImGui_ImplOpenGL3_Init(glsl_version);
}

void update_frame() {
  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();
}

void draw() {
  ImGui::Render();
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

void cleanup() {
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();
}

struct {
  bool show_popup_save = false;
  bool show_popup_open = false;
} MenuBarState;

void text_popup(const char *win_label, const char *input_label, bool &toggle,
                void (*callback)(const char *)) {
  if (toggle) {
    ImGui::OpenPopup(win_label);
  }

  ImVec2 center = ImGui::GetMainViewport()->GetCenter();
  ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
  if (ImGui::BeginPopupModal(win_label, NULL,
                             ImGuiWindowFlags_AlwaysAutoResize)) {
    AppState::get().ignore_keystrokes = true;
    struct Funcs {
      static int MyCallback(ImGuiInputTextCallbackData *data) {
        if (data->EventFlag == ImGuiInputTextFlags_CallbackCompletion) {
          data->InsertChars(data->CursorPos, "..");
        }
        return 0;
      }
    };
    static const int max_filename_length = 128;
    static char buf1[max_filename_length];
    ImGui::InputText(input_label, buf1, max_filename_length,
                     ImGuiInputTextFlags_CallbackCompletion, Funcs::MyCallback);
    if (!ImGui::IsAnyItemActive() && !ImGui::IsMouseClicked(0))
      ImGui::SetKeyboardFocusHere(0);
    if (ImGui::Button("OK", ImVec2(120, 0))) {
      ImGui::CloseCurrentPopup();
      toggle = false;
      // PointCloud::save_pointclouds(buf1);
      callback(buf1);
      for (int i = 0; i < max_filename_length; i++)
        buf1[i] = 0;
    }
    ImGui::SetItemDefaultFocus();
    ImGui::SameLine();
    if (ImGui::Button("Cancel", ImVec2(120, 0))) {
      ImGui::CloseCurrentPopup();
      toggle = false;
      for (int i = 0; i < max_filename_length; i++)
        buf1[i] = 0;
    }
    ImGui::EndPopup();
  } else {
    AppState::get().ignore_keystrokes = false;
  }
}

static void ShowMenuFile() {
  ImGui::MenuItem("Point Cloud I/O", NULL, false, false);
  if (ImGui::MenuItem("New")) {
    reset_scene(DrawNode::scene);
  }
  if (ImGui::MenuItem("Open", "Ctrl+O")) {
    MenuBarState.show_popup_open = true;
  }
  if (ImGui::MenuItem("Save As..", "CTRL+S")) {
    MenuBarState.show_popup_save = true;
  }
}

static void update_menu_bar() {
  if (ImGui::BeginMainMenuBar()) {
    if (ImGui::BeginMenu("File")) {
      gui::ShowMenuFile();
      ImGui::EndMenu();
    }
    ImGui::EndMainMenuBar();

    text_popup(
        "Save As..", "Filename", MenuBarState.show_popup_save,
        [](const char *filename) { PointCloud::save_pointclouds(filename); });

    text_popup("Open", "Filename", MenuBarState.show_popup_open,
               [](const char *filename) {
                 PointCloud::read_pointclouds(filename, PointCloud::all);
                 load_scene();
               });
  }
}

static void update_drawnode_menu(DrawNode *node) {
  char buf1[124];
  snprintf(buf1, sizeof(buf1), "ID: %d, children: %d", node->id,
           node->countChildrenRecursive());
  char buf3[124];
  snprintf(buf3, sizeof(buf3), "##%d", node->id);
  ImGui::Checkbox(buf3, &node->visible);
  ImGui::SameLine();
  if (ImGui::TreeNode(buf1)) {
    for (unsigned i = 0; i < node->meshes.size(); i++) {
      char buf2[124];
      snprintf(buf2, sizeof(buf2), "Mesh(%d): verts: %zu", i,
               node->meshes[i]->verts.size() / node->meshes[i]->stride);
      ImGui::Text(buf2);
    }

    for (UniformBase *u : node->uniforms) {
      char buf2[124];
      snprintf(buf2, sizeof(buf2), "Uniform(%s)", u->name.c_str());
      ImGui::Text(buf2);
    }

    for (DrawNode *c : node->children) {
      update_drawnode_menu(c);
    }
    ImGui::TreePop();
  }
}

static void update_window_settings() {
  static bool first = true;

  if (AppState::get().gui_windows_need_update)
    ImGui::SetNextWindowPos(
        ImVec2(ImGui::GetMainViewport()->GetCenter().x * 2, 20), 0,
        ImVec2(1, 0));

  ImGui::Begin("Settings");
  AppState::get().window_settings_hovered = ImGui::IsWindowHovered();

  ImGui::Text("Average %.3f ms/frame (%.1f FPS)",
              1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

  if (first) {
    ImGui::SetNextItemOpen(1);
    first = false;
  }

  if (ImGui::TreeNode("TFMini Lidar")) {
    ImGui::SliderInt("Scan Range X", (int *)&(AppState::get().scan_range.x), 0,
                     200);
    ImGui::SliderInt("Scan Range Y", (int *)&(AppState::get().scan_range.y),
                     750, 1400);

    if (ImGui::Button("Force Scan"))
      AppState::get().should_scan = true;

    ImGui::TreePop();
  }

  if (ImGui::TreeNode("Servo Arm")) {

    ImGui::SliderFloat3("target", (float *)&AppState::get().arm_target, -4.0,
                        4.0, "%.2f", 0);

    ImGui::TreePop();
  }

  if (ImGui::TreeNode("Render Tree")) {
    DrawNode *n = &DrawNode::scene;
    update_drawnode_menu(n);
    ImGui::TreePop();
  }
  ImGui::End();
}

static void update_cam_window(const gl::Texture2D &cam_tex) {
  static int car_angle_deg = 0;
  static int car_angle_deg_prev = 0;
  static bool needs_update = false;

  // static std::thread car_turn_thread([&]() {
  //   while (true) {
  //     if (needs_update) {
  //       car_turn(esp_sock, car_angle_deg - car_angle_deg_prev);
  //       car_angle_deg_prev = car_angle_deg;
  //       needs_update = false;
  //     } else {
  //       time_sleep(0.1);
  //     }
  //   }
  // });

  if (cam_tex.height > 0) {

    struct WindowTextureConstraints {
      static void tex_ratio(ImGuiSizeCallbackData *data) {
        gl::Texture2D *tex = (gl::Texture2D *)data->UserData;
        float ratio = (float)tex->width / tex->height;
        data->DesiredSize.x = (data->DesiredSize.y - 75) * ratio;
      }
    };

    if (AppState::get().gui_windows_need_update) {
      ImGui::SetNextWindowPos(ImVec2(0, 0), 0, ImVec2(0, 0));
    }

    ImGui::SetNextWindowSizeConstraints(ImVec2(0, 0), ImVec2(FLT_MAX, FLT_MAX),
                                        WindowTextureConstraints::tex_ratio,
                                        (void *)&cam_tex);

    ImGui::Begin("camera");
    ImVec2 vMin = ImGui::GetWindowContentRegionMin();
    ImVec2 vMax = ImGui::GetWindowContentRegionMax();
    ImVec2 winsize(vMax.x - vMin.x, vMax.y - vMin.y - 75);
    ImGui::Image((ImTextureID)cam_tex.id, winsize);

    ImGui::SliderInt2("pan/tilt", (int *)&AppState::get().cam_servo_width.x,
                      500, 2500);
    if (ImGui::SliderInt("car angle", (int *)&car_angle_deg, -180, 180)) {
      needs_update = true;
    }

    if (ImGui::Button("grab")) {
      AppState::get().should_send_coords = true;
    }
    ImGui::End();
  }
}

} // namespace gui
} // namespace xn