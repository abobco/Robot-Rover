#include "xn_rover.hpp"
#include "../app/app_state.hpp"
#include "../app/app_threads_shared.hpp"

namespace xn {

void Rover::lidar_scan(std::vector<std::vector<glm::vec3>> &nav_verts,
                       xn::GridGraph &navgraph) {
  int servo_step = 50;
  int step_incr = 1, step_max = 100;

  PointCloud::all.push_back(PointCloud(glm::vec3(position.x, 0, position.y)));
  PointCloud &pc_new = PointCloud::all.back();

  lidar_scan_ex(pc_new, servo_step, step_incr, step_max);

  turn(180);

  lidar_scan_ex(pc_new, servo_step, step_incr, step_max);

  turn(-180);

  if (PointCloud::all.size() > 1) {
    lidar_scan_error_correct(pc_new,
                             PointCloud::all[PointCloud::all.size() - 2]);
  }
  lidar_scan_postprocess(pc_new, nav_verts, navgraph);

  // update_graph_boxes();
  AppState::get().graph_needs_update = true;
}

void Rover::lidar_scan_error_correct(PointCloud &pc_new,
                                     const PointCloud &pc_prev,
                                     int iterations) {
  for (int i = 0; i < iterations; i++) {
    glm::vec3 tx(0);
    float orig_ang = rotation;
    PointCloud::hillclimb_transform(pc_prev, pc_new, tx, rotation);
    AppState::get().scan_needs_update = true;
    DUMP(tx);
    DUMP(rotation);

    turn(glm::degrees(orig_ang - rotation));
    position.x += tx.x;
    position.y += tx.z;
  }
}

void Rover::lidar_scan_postprocess(
    const PointCloud &pc, std::vector<std::vector<glm::vec3>> &nav_verts,
    GridGraph &navgraph) {
  // fan triangulation
  std::vector<glm::vec3> triverts;
  pc.toNavmesh(triverts);
  nav_verts.push_back(triverts);

  // convert to pathfinding grid
  navgraph.cells.clear();
  PointCloud::march_squares(PointCloud::all, 50, 50, navgraph.boxSize, position,
                            navgraph.cells, navgraph.offset, 10, 2);

  navgraph.graph = preprocess_graph(navgraph.cells);
}

void Rover::lidar_scan_ex(PointCloud &pc_new, int servo_step, int step_incr,
                          int step_max) {
  int step = 0;

  const LidarReadInstruction l_read;
  LidarStepInstruction x_step;
  x_step.dir = 0;
  LidarServoInstruction servo_ctl;
  servo_ctl.width = scan_range.x;
  send_pack(esp32, servo_ctl);

  int burst_size = 50;
  int pack_size = burst_size * sizeof(LidarReadInstruction) +
                  burst_size * sizeof(LidarStepInstruction) +
                  2 * burst_size * sizeof(uint32_t);
  DUMP(pack_size);
  char *bbuf = new char[pack_size];
  int bpos = 0;
  for (int i = 0; i < burst_size; i++) {
    bpos += pack_instruction(x_step, &(bbuf[bpos]));
    bpos += pack_instruction(l_read, &(bbuf[bpos]));
  }

  while (servo_ctl.width < scan_range.y) {
    if (step + step_incr > step_max || step + step_incr < 0) {
      step_incr *= -1;
      x_step.dir = !x_step.dir;
      servo_ctl.width += servo_step;
      send_pack(esp32, servo_ctl);
      bpos = 0;
      for (int i = 0; i < burst_size; i++) {
        bpos += pack_instruction(x_step, &(bbuf[bpos]));
        bpos += pack_instruction(l_read, &(bbuf[bpos]));
      }
      continue;
    }

    send(esp32, bbuf, pack_size, 0);
    for (int i = 0; i < burst_size; i++) { // read lidar
      uint16_t d = 0;
      int acks = 0;
      acks += wait_ack(esp32);
      // printf("ack1 ");
      acks += wait_ack(esp32);
      // printf("ack2\n");
      if (acks < 2) {
        printf("resending remaining instructions\n");
        size_t bsent =
            i * (sizeof(LidarReadInstruction) + sizeof(LidarStepInstruction) +
                 2 * sizeof(uint32_t));
        printf("%ld sent, %ld remaining\n", bsent, pack_size - bsent);
        send(esp32, &bbuf[bsent], pack_size - bsent, 0);
        --i;
        continue;
      }
      read_int<uint16_t>(esp32, d, false);

      step += step_incr;
      // DUMP(step);

      if (d == 65535)
        continue;
      float servo_ang_f = ((servo_ctl.width - scan_range.x) / 2000.0f) * M_PI;
      double ang = -(step * (1.0 / 100) * M_PI);

      vec3 v = {(float)d * 0.01f, 0, 0};
      v = vec3::rotate_axis(v, {0, 1, 0}, ang + rotation);
      vec3 rot_ax2 = vec3::rotate_axis({0, 0, 1}, {0, 1, 0}, ang);
      v = vec3::rotate_axis(v, rot_ax2, servo_ang_f);

      // DUMP(v);
      pc_new.points.push_back(
          glm::vec3(v.x + position.x, v.y, v.z + position.y));
    }
  }
  printf("scan done\n");
}

int Rover::move_forward(float gridsquares, float gridbox_size) {
  float c = M_PI * wheelDiameter;
  MotorInstruction cmd_m_forward;
  cmd_m_forward.length_pulses =
      (100 * gridsquares * gridbox_size) / (c / motorCpr);
  cmd_m_forward.dir_left = 0;
  cmd_m_forward.dir_right = 0;

  printf("sending motor instruction\n");
  // DUMP(cmd_m_forward.length_pulses);
  return send_pack(esp32, cmd_m_forward);
}

int Rover::turn(int deg) {
  // deg / 360 = x / c
  float d = wheelSeparation * M_PI * abs(deg) / 360;
  float c = M_PI * wheelDiameter;
  // MOT_CPR / WHEEL_DIAM = x / c
  // x = c*MOT_CPR / 11
  MotorInstruction cmd_m_right;
  cmd_m_right.length_pulses = d / (c / wheelSeparation);
  if (deg > 0) {
    cmd_m_right.dir_left = 1;
    cmd_m_right.dir_right = 0;
  } else {
    cmd_m_right.dir_left = 0;
    cmd_m_right.dir_right = 1;
  }
  printf("sending motor instruction\n");
  int status = send_pack(esp32, cmd_m_right);
  if (!status)
    return 0;
  time_sleep(2 * (float)abs(deg) / 90);
  rotation += glm::radians((float)deg);
  return 1;
}

void Rover::follow_path(std::vector<std::vector<glm::vec3>> &nav_verts,
                        GridGraph &navgraph) {
  static glm::vec2 heading(-1, 0);
  const int rescan_interval = 4;
  DUMP(navgraph.path.size());

  int ii = 0;
  if (navgraph.path.size() != 0)
    for (auto i = navgraph.path.begin(); i < navgraph.path.end() - 1; i++) {
      if (ii == rescan_interval) {
        ii = 0;

        lidar_scan(nav_verts, navgraph);

        glm::vec3 start_world(position.x, 0, position.y);
        glm::vec2 start =
            world_to_grid(start_world, navgraph.boxSize, navgraph.offset);
        glm::ivec2 target_cell =
            world_to_grid(target, navgraph.boxSize, navgraph.offset);
        navgraph.path = A_star(navgraph.graph, start.x, start.y, target_cell.x,
                               target_cell.y);
        // i = navgraph.path.begin();
        follow_path(nav_verts, navgraph);
        return;
      }

      GridNode &current = *i;
      GridNode &next = *(i + 1);
      glm::vec2 heading_new(next.x - current.x, next.y - current.y);
      float cross_res =
          glm::cross(glm::vec3(heading_new, 0), glm::vec3(heading, 0)).z;
      int status = 1;

      if (abs(glm::dot(heading_new, heading) + 1) < SMEPSILON) {
        printf("behind\n");
        status = turn(180);
      } else if (abs(cross_res - 1) < SMEPSILON) {
        printf("left\n");
        status = turn(90);
      } else if (abs(cross_res + 1) < SMEPSILON) {
        printf("right\n");
        status = turn(-90);
      } else {
        status = printf("ahead\n");
      }

      if (!status)
        return;

      int extra_square_count = 0;
      for (auto j = i + 2; j < navgraph.path.end(); j++) {
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

      status = move_forward(1 + extra_square_count, navgraph.boxSize);
      if (!status)
        return;
      time_sleep(3 + extra_square_count * 2);
      heading = heading_new;
      position += heading * navgraph.boxSize * (1.0f + extra_square_count);
    }

  if (heading == glm::vec2(0, 1))
    rotation = M_PI_2;
  else if (heading == glm::vec2(0, -1))
    rotation = -M_PI_2;
  else if (heading == glm::vec2(1, 0))
    rotation = M_PI;
  else if (heading == glm::vec2(-1, 0))
    rotation = 0;
}

// void wifi_car_test(int &sockfd, float t = 1) {
//   PointCloud p;
//   for (int i = 0; i < 200; i++) {
//     float ang = i * M_PI * 2.0f / 200.0f;
//     glm::vec3 pt =
//         glm::vec3(car_position.x + cos(ang), 0, car_position.y + sin(ang));
//     p.points.push_back(pt);
//     p.points.push_back(pt);
//   }

//   std::vector<glm::vec3> triverts;
//   p.toNavmesh(triverts);
//   nav_verts.push_back(triverts);
//   PointCloud::all.clear();
//   PointCloud::all.push_back(p);

//   // convert to pathfinding grid
//   gridgraph.clear();
//   PointCloud::march_squares(PointCloud::all, 50, 50, navgraph.boxSize,
//   car_position,
//                             gridgraph, navgraph.offset, 10, 2);

//   graph = preprocess_graph(gridgraph);
// }
} // namespace xn