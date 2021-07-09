#include "../xn_robot_host_app.hpp"
#include "json.hpp"
#include <fstream>
#include <iostream>
#include <streambuf>
#include <util/xn_json.hpp>

// using json = nlohmann::json;

namespace xn {
// json load_json_file(const std::string &filepath) {
//   std::ifstream t(filepath);
//   std::string buf((std::istreambuf_iterator<char>(t)),
//                   std::istreambuf_iterator<char>());
//   json j = json::parse(buf);
//   t.close();
//   return j;
// }

json load_settings_json(std::string filename) {
  json j = load_json_file(filename);

  // AppState::get().WHEEL_DIAM = j["wheel_diameter"];
  // AppState::get().WHEEL_SEPARATION = j["wheel_separation"];

  AppState::get().gridbox_size = j["gridbox_size"];

  AppState::get().cam_servo_width.x = j["cam_servo_width.x"];
  AppState::get().cam_servo_width.y = j["cam_servo_width.y"];

  return j;
}
} // namespace xn