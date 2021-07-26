/**
 * Globals and (hopefully) threadsafe getters
 **/

#pragma once
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
// clang-format off
#include "../robo/xn_net.hpp"
#include "../robo/xn_console.hpp"
#include "../robo/xn_search.hpp"
#include <fstream>
#include <glm/glm.hpp>
#include <mutex>
// clang-format on

namespace xn {

struct SshConsole;

class AppState {
private:
  AppState() {}

public:
  AppState(AppState const &) = delete;
  void operator=(AppState const &) = delete;
  static AppState &AppState::get();

  std::mutex mut, frame_mut, rpi_log_mut, path_mut, arm_mut;

  bool run = 1;
  bool sim_rover = true;
  bool pic_needs_update = false;
  bool conn_accepted = false;
  bool should_scan = false;
  bool ignore_keystrokes = false;
  bool should_restart_esp = false;
  bool window_settings_hovered = false;
  bool graph_needs_update = false;
  bool scan_needs_update = false;
  bool should_send_coords = false;
  bool car_following_path = false;
  bool should_draw_boxes = false;
  bool path_needs_update = false;
  bool yolo_completed = false;
  bool received_pic = false;
  bool move_to_target = false;
  bool grab_trigger = false;
  bool update_sim_grab_target = false;
  bool found_chessboard = false;
  bool gui_windows_need_update = true;
  bool dirty_points;
};

} // namespace xn