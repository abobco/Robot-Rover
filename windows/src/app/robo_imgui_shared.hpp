#pragma once
#include "../robo/xn_console.hpp"
#include "../robo/xn_pointcloud.hpp"
#include "app_threads_shared.hpp"
#include "imgui.h"

namespace xn {

class RobotSettingsWindow {
  bool firstDraw = true;

public:
  void (*userCallback)(void *argv);

  void setUserCallback(void (*userCallback)(void *argv)) {
    this->userCallback = userCallback;
  }

  void draw(RobotController &robot, void *userCallbackData = NULL) {
    if (AppState::get().gui_windows_need_update)
      ImGui::SetNextWindowPos(
          ImVec2(ImGui::GetMainViewport()->GetCenter().x * 2, 20), 0,
          ImVec2(1, 0));

    ImGui::Begin("Settings");
    AppState::get().window_settings_hovered = ImGui::IsWindowHovered();

    ImGui::Text("Average %.3f ms/frame (%.1f FPS)",
                1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

    if (firstDraw) {
      ImGui::SetNextItemOpen(1);
      firstDraw = false;
    }

    if (ImGui::TreeNode("TFMini Lidar")) {
      ImGui::SliderInt("Scan Range X", (int *)&(robot.rover.scan_range.x), 0,
                       200);
      ImGui::SliderInt("Scan Range Y", (int *)&(robot.rover.scan_range.y), 750,
                       1400);
      if (ImGui::Button("Force Scan"))
        AppState::get().should_scan = true;

      ImGui::TreePop();
    }

    if (ImGui::TreeNode("Servo Arm")) {

      ImGui::SliderFloat3("target", (float *)&robot.arm.target, -4.0, 4.0,
                          "%.2f", 0);

      ImGui::TreePop();
    }

    userCallback(userCallbackData);

    ImGui::End();
  }
};

struct TextureWindow {
  static void draw(ImTextureID id, ImVec2 texture_dimensions,
                   RobotController &robot) {
    static int car_angle_deg = 0;
    static int car_angle_deg_prev = 0;
    static bool needs_update = false;

    if (texture_dimensions.y <= 0 || texture_dimensions.x <= 0)
      return;

    auto tex_ratio = [](ImGuiSizeCallbackData *data) {
      ImVec2 *dims = (ImVec2 *)data->UserData;
      float ratio = (float)dims->x / dims->y;
      data->DesiredSize.x = (data->DesiredSize.y - 96) * ratio;
    };

    if (AppState::get().gui_windows_need_update) {
      ImGui::SetNextWindowPos(ImVec2(0, 0), 0, ImVec2(0, 0));
    }

    ImGui::SetNextWindowSizeConstraints(ImVec2(0, 0), ImVec2(FLT_MAX, FLT_MAX),
                                        tex_ratio, (void *)&texture_dimensions);

    ImGui::Begin("camera");
    ImVec2 vMin = ImGui::GetWindowContentRegionMin();
    ImVec2 vMax = ImGui::GetWindowContentRegionMax();
    ImVec2 winsize(vMax.x - vMin.x, vMax.y - vMin.y - 75);
    ImGui::Image(id, winsize);

    ImGui::SliderInt2("pan/tilt", (int *)&robot.cam_servo_width.x, 500, 2500);
    if (ImGui::SliderInt("car angle", (int *)&car_angle_deg, -180, 180)) {
      needs_update = true;
    }

    if (ImGui::Button("grab")) {
      AppState::get().should_send_coords = true;
    }
    ImGui::SameLine();
    if (ImGui::Button("sim grab")) {
      if (AppState::get().conn_accepted)
        AppState::get().update_sim_grab_target = true;
      else
        AppState::get().grab_trigger = true;
    }

    ImGui::SliderFloat3("arm_target", (float *)&robot.arm.target, -1.0, 1.0);
    ImGui::End();
  }
};
} // namespace xn