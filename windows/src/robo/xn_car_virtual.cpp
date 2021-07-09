#include "xn_car_virtual.hpp"
#include "../app/app_threads_shared.hpp"

namespace xn {

int car_move_forward_sim(glm::vec2 &car_pos, glm::vec2 &heading, float d) {
  //   vec3 v{-1, 0, 0};
  //   v = vec3::rotate_axis(v, {0, 1, 0}, (float)angle);

  //   glm::vec2 t(v.toGLM().x, v.toGLM().z);
  car_pos += heading * d;

  return 1;
}

int car_turn_sim(float &car_angle, int deg) {
  car_angle += glm::radians((float)deg);
  return 1;
}

void car_follow_path_sim(CarInfo & info, std::vector<GridNode> & path
                           /* ,std::vector<std::pair<float, glm::vec2>> &
                               error_list*/) {
  static glm::vec2 heading(-1, 0);
  const int rescan_interval = 4;
  //   DUMP(path.size());

  int ii = 0;
  if (path.size() != 0)
    for (auto i = path.begin(); i < path.end() - 1; i++) {
      if (ii == rescan_interval) {
        ii = 0;

        glm::vec3 start_world(info.pos.x, 0, info.pos.y);
        glm::ivec2 start =
            world_to_grid(start_world, info.gridbox_size, info.grid_offset);
        // BUG HERE?
        // glm::ivec2 target =
        //     world_to_grid(info.target_world,info.gridbox_size,info.grid_offset);

        // THIS IS CHEATING
        // glm::ivec2 start(i->x, i->y);
        glm::ivec2 target(path.back().x, path.back().y);

        path = A_star(info.graph, start.x, start.y, target.x, target.y);
        AppState::get().path_needs_update = true;
        car_follow_path_sim(info, path);
        return;
      }

      GridNode &current = *i;
      GridNode &next = *(i + 1);
      glm::vec2 heading_new(next.x - current.x, next.y - current.y);
      float cross_res =
          glm::cross(glm::vec3(heading_new, 0), glm::vec3(heading, 0)).z;
      int status = 1;

      if (abs(glm::dot(heading_new, heading) + 1) < SMEPSILON) {
        xn_printf("behind\n");
        status = car_turn_sim(info.angle, 180);
      } else if (abs(cross_res - 1) < SMEPSILON) {
        xn_printf("left\n");
        status = car_turn_sim(info.angle, 90);
      } else if (abs(cross_res + 1) < SMEPSILON) {
        xn_printf("right\n");
        status = car_turn_sim(info.angle, -90);
      } else {
        xn_printf("ahead\n");
      }

      int extra_square_count = 0;
      for (auto j = i + 2; j < path.end(); j++) {
        glm::vec2 heading_next(j->x - current.x, j->y - current.y);
        heading_next = glm::normalize(heading_next);
        if (abs(1.0f - glm::dot(heading_next, heading_new)) < SMEPSILON) {
          extra_square_count++;
        } else {
          break;
        }
      }
      if (ii + extra_square_count + 1 > rescan_interval) {
        extra_square_count = rescan_interval - 1 - ii;
        ii = rescan_interval;
      } else {
        ii += 1 + extra_square_count;
      }
      i += extra_square_count;
      heading = heading_new;
      car_move_forward_sim(info.pos, heading,
                           (1.0f + extra_square_count) * info.gridbox_size);

      time_sleep(0.5);
      //   info.pos += heading * info.gridbox_size * (1.0f +
      //   extra_square_count);
    }

  if (heading == glm::vec2(0, 1))
    info.angle = (float)M_PI_2;
  else if (heading == glm::vec2(0, -1))
    info.angle = (float)-M_PI_2;
  else if (heading == glm::vec2(1, 0))
    info.angle = (float)M_PI;
  else if (heading == glm::vec2(-1, 0))
    info.angle = 0;
}

// void lidar_scan_ex_sim(Shader &s, BoxMesh &b, PointCloud &pc_new,
//                        DrawNode *scan_node, int min_width, int max_width,
//                        int servo_step, int step_incr, int step_max,
//                        const glm::vec2 &offset = glm::vec2(0, 0),
//                        float angle = 0) {
//   int step = 0;
//   int burst_size = 50;
//   while (servo_ctl.width < max_width) {
//     if (step + step_incr > step_max || step + step_incr < 0) {
//       step_incr *= -1;
//       x_step.dir = !x_step.dir;
//       servo_ctl.width += servo_step;
//       continue;
//     }

//     for (int i = 0; i < burst_size; i++) { // read lidar
//       uint16_t d = 0;

//       float r = (float)(rand()) / (float)(RAND_MAX);

//       d = r * 300;

//       step += step_incr;

//       float servo_ang_f =
//           ((servo_ctl.width - min_width) / 2000.0f) * (float)M_PI;
//       double ang = -(step * (1.0 / 100) * M_PI);

//       vec3 v = {(float)d * 0.01f, 0, 0};
//       v = vec3::rotate_axis(v, {0, 1, 0}, (float)ang + angle);
//       vec3 rot_ax2 = vec3::rotate_axis({0, 0, 1}, {0, 1, 0}, (float)ang);
//       v = vec3::rotate_axis(v, rot_ax2, servo_ang_f);

//       mut.lock();
//       pc_new.points.push_back(glm::vec3(v.x + offset.x, v.y, v.z +
//       offset.y));

//       DrawNode *c;
//       if (XN_LIT) {
//         c = new DrawNode(&s, &b);
//         c->addUniform("objectColor",
//                       glm::vec3(gl::sample_colors[0].x,
//                       gl::sample_colors[0].y,
//                                 gl::sample_colors[0].z));
//       } else {
//         c = new DrawNode(&s, &b);
//         c->addUniform("color", gl::sample_colors[0]);
//       }

//       c->transform(pc_new.points.back(), glm::vec3(0.05, 0.05, 0.05));
//       scan_node->addChild(c);
//       mut.unlock();
//     }
//   }
//   xn_printf("scan done\n");
// }

// void lidar_scan_sim(Shader &s, Boxmesh &b,
//                     const glm::vec2 &offset = glm::vec2(0, 0),
//                     float angle = 0) {
//   int min_width = 700, max_width = AppState::get().scan_range.y, servo_step =
//   50; int step_incr = 1, step_max = AppState::get().scan_range.x;

//   PointCloud::all.push_back(PointCloud(glm::vec3(offset.x, 0, offset.y)));
//   PointCloud &pc_new = PointCloud::all.back();

//   DrawNode *scan_node = draw_map["scan"]->addChild(new DrawNode());

//   lidar_scan_ex_sim(s, b, pc_new, scan_node, min_width, max_width,
//   servo_step,
//                     step_incr, step_max, offset, angle);

//   car_turn_sim(180);
//   angle += (float)M_PI;

//   lidar_scan_ex_sim(s, b, pc_new, scan_node, min_width, max_width,
//   servo_step,
//                     step_incr, step_max, offset, angle);

//   car_turn_sim(-180);

//   if (PointCloud::all.size() > 1) {
//     lidar_scan_error_correct_sim(
//         pc_new, PointCloud::all[PointCloud::all.size() - 2], scan_node);
//   }
//   lidar_scan_postprocess(pc_new);

//   // update_graph_boxes();
//   AppState::get().graph_needs_update = true;
// }
} // namespace xn