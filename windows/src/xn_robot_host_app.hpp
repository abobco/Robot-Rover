/*
    Car dims: 25.5cm x 18cm
    lidar distance from center: 11
*/

#pragma once

// #include "xn_net.hpp"
#include "app/app_state.hpp"
#include "app/app_threads_shared.hpp"
#include "opengl/xn_gl.hpp"
#include "opengl/xn_renderer.hpp"
// #include "robo/xn_gpio.hpp"
#include "robo/xn_ik.hpp"
#include "robo/xn_log.hpp"
#include "robo/xn_pointcloud.hpp"
#include "util/xn_vec.hpp"
#include <arm/xn_gpio.hpp>
#include <unordered_map>


namespace xn {

std::stringstream log_buffer;

// const float WHEEL_DIAM = 9.4f; // with battery
std::unordered_map<std::string, DrawNode *> draw_map;

std::vector<glm::vec3> arm_verts;
std_vec2d<glm::vec3> nav_verts;

Shader color_shader;
Shader lit_shader;
BoxMesh boxmesh;
// ik::ServoChain *armptr;

// pi camera stream
gl::Texture2D cam_tex;

void reset_scene(DrawNode &scene, GridGraph &navgraph) {
  scene = DrawNode();
  PointCloud::all.clear();
  nav_verts.clear();
  navgraph.graph.clear();
  navgraph.gridgraph.clear();
}

void update_graph_boxes() {
  if (draw_map["graph"] == NULL) {
    draw_map["graph"] = DrawNode::scene.addChild(new DrawNode());
  }
  draw_map["graph"]->children.clear();

  std::vector<glm::vec3> graph_verts;
  for (unsigned i = 0; i < AppState::get().gridgraph.size(); i++)
    for (unsigned j = 0; j < AppState::get().gridgraph[i].size(); j++)
      if (!AppState::get().gridgraph[i][j]) {
        glm::vec3 v(i * AppState::get().gridbox_size, 0,
                    j * AppState::get().gridbox_size);
        v += AppState::get().grid_offset;
        v.y = -AppState::get().gridbox_size * 0.5f - 0.01f;
        graph_verts.push_back(v);
      }
  Mesh *m =
      Mesh::gen_box_batch(graph_verts, glm::vec3(AppState::get().gridbox_size));
  // m->setPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  if (XN_LIT) {
    DrawNode *g = draw_map["graph"]->addChild(new DrawNode(&lit_shader, m));
    g->addUniform("objectColor", glm::vec3(0.5f, 0.5f, 0.5f));
  } else {
    DrawNode *g = draw_map["grap"]->addChild(new DrawNode(&color_shader, m));
    g->addUniform("color", glm::vec4(0.5f, 0.5f, 0.5f, 1.0f));
  }
}

void gen_navgraph(GridGraph &navgraph) {
  double s = glfwGetTime();
  PointCloud::march_squares(PointCloud::all, 50, 50,
                            AppState::get().gridbox_size,
                            AppState::get().car_position, navgraph.cells,
                            AppState::get().grid_offset, 10, 2);
  xn_printf("marching squares took: %.2f s\n", glfwGetTime() - s);

  s = glfwGetTime();
  navgraph.graph = preprocess_graph(navgraph.cells);
  // pthread_mutex_unlock(&mut);
  xn_printf("graph processing took: %.2f s\n", glfwGetTime() - s);
}

void load_scene() {
  draw_map["axis"] = DrawNode::scene.addChild(new DrawNode());
  for (int i = 0; i < 3; i++) {
    std::vector<float> verts;
    verts.reserve(6);
    glm::vec3 v1(0);
    glm::vec3 v2(0);
    v2[i] = 1;

    glm::vec4 col(0);
    col[i] = 1;
    col.w = 1;

    verts.insert(verts.end(), (float *)&v1, (float *)&v1 + 3);
    verts.insert(verts.end(), (float *)&v2, (float *)&v2 + 3);
    Mesh *m = new Mesh(verts);
    m->draw_mode = GL_LINES;
    m->polygon_mode.second = GL_LINE;
    DrawNode *n = draw_map["axis"]->addChild(new DrawNode(&color_shader, m));
    n->addUniform("color", col);
  }
  draw_map["scan"] = DrawNode::scene.addChild(new DrawNode());
  for (auto &p : PointCloud::all) {
    if (p.points.size() == 0)
      continue;
    if (XN_LIT) {
      DrawNode *c = draw_map["scan"]->addChild(
          new DrawNode(&lit_shader, Mesh::gen_box_batch(p.points)));
      glm::vec3 col(gl::sample_colors[0].x, gl::sample_colors[0].y,
                    gl::sample_colors[0].z);
      c->addUniform("objectColor", col);
    } else {
      DrawNode *c = draw_map["scan"]->addChild(
          new DrawNode(&color_shader, Mesh::gen_box_batch(p.points)));
      c->addUniform("color", gl::sample_colors[0]);
    }
  }
  // grid init
  draw_map["grid"] = DrawNode::scene.addChild(new DrawNode(
      &color_shader, Mesh::gen_grid(AppState::get().gridbox_size, 100, 100)));
  draw_map["grid"]->addUniform("color", glm::vec4(1.0f, 1.0f, 1.0f, 0.6f));
  draw_map["grid"]->transform(AppState::get().grid_offset -
                              glm::vec3(AppState::get().gridbox_size * 0.5, 0,
                                        AppState::get().gridbox_size * 0.5));

  std::vector<float> triverts;
  triverts.insert(triverts.begin(), std::begin(gl::tri_verts_eq),
                  std::end(gl::tri_verts_eq));
  Mesh *triangle = new Mesh(triverts);
  draw_map["car"] =
      DrawNode::scene.addChild(new DrawNode(&color_shader, triangle));
  draw_map["car"]->addUniform("color", glm::vec4(1.0f, 1.0f, 0.2f, 1.0f));

  gen_navgraph();
  delete draw_map["graph"];
  draw_map["graph"] = NULL;
  update_graph_boxes();

  draw_map["navmesh"] = DrawNode::scene.addChild(new DrawNode());

  DUMP(PointCloud::all.size());
  for (auto &p : PointCloud::all) {
    nav_verts.push_back(std::vector<glm::vec3>());
    p.toNavmesh(nav_verts.back());
    DrawNode *n = draw_map["navmesh"]->addChild(
        new DrawNode(&color_shader, new Mesh(nav_verts.back())));
    n->addUniform("color", glm::vec4(0.8f, 0.6f, 0.4f, 0.5f));
  }

  draw_map["arm"] = DrawNode::scene.addChild(new DrawNode(&color_shader));
  draw_map["arm"]->addUniform("color", glm::vec4(1.0f, 1.0f, 0.2f, 0.5f));
  for (int i = 0; i < getArm()->ideal_chain.bone_count; i++) {
    DrawNode *n =
        draw_map["arm"]->addChild(new DrawNode(&color_shader, &boxmesh));
    n->transform(getArm()->positions[i].toGLM(), glm::vec3(0.1f));
    arm_verts.push_back(getArm()->positions[i].toGLM());
  }

  draw_map["arm_skeleton"] = draw_map["arm"]->addChild(
      new DrawNode(&color_shader, new Mesh(arm_verts)));
  draw_map["arm_skeleton"]->meshes[0]->draw_mode = GL_LINE_STRIP;
  draw_map["arm_skeleton"]->addUniform("color",
                                       glm::vec4(1.0f, 1.0f, 0.2f, 1.0f));
  draw_map["arm_target"] =
      draw_map["arm"]->addChild(new DrawNode(&color_shader, &boxmesh));
  draw_map["arm_target"]->visible = false;
  draw_map["arm_target"]->addUniform("color",
                                     glm::vec4(0.2f, 0.2f, 1.0f, 0.5f));
}

int car_move_forward(SOCKET &sockfd, float gridsquares, float gridbox_size,
                     float wheel_diameter, unsigned motor_cpr) {
  float c = (float)M_PI * wheel_diameter;
  MotorInstruction cmd_m_forward;
  cmd_m_forward.length_pulses =
      (uint32_t)(100 * gridsquares * gridbox_size / (c / motor_cpr));
  cmd_m_forward.dir_left = 0;
  cmd_m_forward.dir_right = 0;

  xn_printf("sending motor instruction\n");
  return send_pack(sockfd, cmd_m_forward);
}

int car_turn(SOCKET &sockfd, float wheel_diameter, float wheel_separation,
             unsigned motor_cpr, int deg = 90) {
  // deg / 360 = x / c
  float d = wheel_separation * (float)M_PI * abs(deg) / 360;
  float c = (float)M_PI * wheel_diameter;
  // MOT_CPR / WHEEL_DIAM = x / c
  // x = c*MOT_CPR / 11
  MotorInstruction cmd_m_right;
  cmd_m_right.length_pulses = (uint32_t)(d / (c / motor_cpr));
  if (deg > 0) {
    cmd_m_right.dir_left = 1;
    cmd_m_right.dir_right = 0;
  } else {
    cmd_m_right.dir_left = 0;
    cmd_m_right.dir_right = 1;
  }
  xn_printf("sending motor instruction\n");
  int status = send_pack(sockfd, cmd_m_right);
  if (!status)
    return 0;
  time_sleep(2 * (float)abs(deg) / 90);
  AppState::get().car_angle += glm::radians((float)deg);
  return 1;
}

void lidar_scan_error_correct(SOCKET &sockfd, PointCloud &pc_new,
                              const PointCloud &pc_prev, DrawNode *scan_node,
                              float wheel_diameter, float wheel_separation,
                              unsigned motor_cpr, int iterations = 2) {
  xn_printf("begin error correction: %.2f\n", glfwGetTime());
  for (int i = 0; i < iterations; i++) {
    glm::vec3 tx(0);
    float orig_ang = AppState::get().car_angle;
    double start_time = glfwGetTime();
    PointCloud::hillclimb_transform(pc_prev, pc_new, tx,
                                    AppState::get().car_angle);
    AppState::get().scan_needs_update = true;
    DUMP(tx);
    DUMP(AppState::get().car_angle);

    xn_printf("hillclimb %d took: %.2f s\n", i, glfwGetTime() - start_time);
    car_turn(sockfd, wheel_diameter, wheel_separation, motor_cpr,
             (int)glm::degrees(orig_ang - AppState::get().car_angle));
    AppState::get().car_position.x += tx.x;
    AppState::get().car_position.y += tx.z;
  }
}

void lidar_scan_postprocess(const PointCloud &pc, GridGraph &navgraph) {
  // fan triangulation
  double s = glfwGetTime();
  std::vector<glm::vec3> triverts;
  pc.toNavmesh(triverts);

  nav_verts.push_back(triverts);
  // xn_printf("triangulation took: %.2f s\n", glfwGetTime() - s);

  // convert to pathfinding grid
  s = glfwGetTime();
  AppState::get().gridgraph.clear();
  PointCloud::march_squares(PointCloud::all, 50, 50,
                            AppState::get().gridbox_size,
                            AppState::get().car_position, navgraph.cells,
                            AppState::get().grid_offset, 10, 2);
  xn_printf("marching squares took: %.2f s\n", glfwGetTime() - s);

  s = glfwGetTime();
  navgraph.graph = preprocess_graph(navgraph.cells);
  xn_printf("graph processing took: %.2f s\n", glfwGetTime() - s);
}

void lidar_scan_ex(PointCloud &pc_new, DrawNode *scan_node, SOCKET &sockfd,
                   int min_width, int max_width, int servo_step, int step_incr,
                   int step_max, const glm::vec2 &offset = glm::vec2(0, 0),
                   float angle = 0) {
  int step = 0;
  // PointCloud::all.push_back(PointCloud(glm::vec3(offset.x, 0, offset.y)));
  // PointCloud &pc_new = PointCloud::all.back();

  // DrawNode *scan_node = DrawNode::scene.addChild(new DrawNode());

  const LidarReadInstruction l_read;
  LidarStepInstruction x_step;
  x_step.dir = 0;
  LidarServoInstruction servo_ctl;
  servo_ctl.width = min_width;
  send_pack(sockfd, servo_ctl);

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

  while (servo_ctl.width < max_width) {
    if (step + step_incr > step_max || step + step_incr < 0) {
      step_incr *= -1;
      x_step.dir = !x_step.dir;
      servo_ctl.width += servo_step;
      send_pack(sockfd, servo_ctl);
      bpos = 0;
      for (int i = 0; i < burst_size; i++) {
        bpos += pack_instruction(x_step, &(bbuf[bpos]));
        bpos += pack_instruction(l_read, &(bbuf[bpos]));
      }
      continue;
    }

    send(sockfd, bbuf, pack_size, 0);
    for (int i = 0; i < burst_size; i++) { // read lidar
      uint16_t d = 0;
      int acks = 0;
      acks += wait_ack(sockfd);
      acks += wait_ack(sockfd);
      if (acks < 2) {
        xn_printf("resending remaining instructions\n");
        size_t bsent =
            i * (sizeof(LidarReadInstruction) + sizeof(LidarStepInstruction) +
                 2 * sizeof(uint32_t));
        xn_printf("%zd sent, %zd remaining\n", bsent, pack_size - bsent);
        send(sockfd, &bbuf[bsent], pack_size - (int)bsent, 0);
        --i;
        continue;
      }
      read_int<uint16_t>(sockfd, d, false);

      step += step_incr;

      if (d == 65535)
        continue;
      float servo_ang_f =
          ((servo_ctl.width - min_width) / 2000.0f) * (float)M_PI;
      double ang = -(step * (1.0 / 100) * M_PI);

      vec3 v = {(float)d * 0.01f, 0, 0};
      v = vec3::rotate_axis(v, {0, 1, 0}, (float)ang + angle);
      vec3 rot_ax2 = vec3::rotate_axis({0, 0, 1}, {0, 1, 0}, (float)ang);
      v = vec3::rotate_axis(v, rot_ax2, servo_ang_f);

      AppState::get().mut.lock();
      pc_new.points.push_back(glm::vec3(v.x + offset.x, v.y, v.z + offset.y));

      DrawNode *c;
      if (XN_LIT) {
        c = new DrawNode(&lit_shader, &boxmesh);
        c->addUniform("objectColor",
                      glm::vec3(gl::sample_colors[0].x, gl::sample_colors[0].y,
                                gl::sample_colors[0].z));
      } else {
        c = new DrawNode(&color_shader, &boxmesh);
        c->addUniform("color", gl::sample_colors[0]);
      }

      c->transform(pc_new.points.back(), glm::vec3(0.05, 0.05, 0.05));
      scan_node->addChild(c);
      AppState::get().mut.unlock();
    }
  }
  xn_printf("scan done\n");
}

void lidar_scan(SOCKET &sockfd, float wheel_diameter, float wheel_separation,
                unsigned motor_cpr, const glm::vec2 &offset = glm::vec2(0, 0),
                float angle = 0) {
  int min_width = 700, max_width = AppState::get().scan_range.y,
      servo_step = 50;
  int step_incr = 1, step_max = AppState::get().scan_range.x;

  PointCloud::all.push_back(PointCloud(glm::vec3(offset.x, 0, offset.y)));
  PointCloud &pc_new = PointCloud::all.back();

  DrawNode *scan_node = draw_map["scan"]->addChild(new DrawNode());

  lidar_scan_ex(pc_new, scan_node, sockfd, min_width, max_width, servo_step,
                step_incr, step_max, offset, angle);

  car_turn(sockfd, wheel_diameter, wheel_separation, motor_cpr, 180);
  angle += (float)M_PI;

  lidar_scan_ex(pc_new, scan_node, sockfd, min_width, max_width, servo_step,
                step_incr, step_max, offset, angle);

  car_turn(sockfd, wheel_diameter, wheel_separation, motor_cpr, -180);

  if (PointCloud::all.size() > 1) {
    lidar_scan_error_correct(
        sockfd, pc_new, PointCloud::all[PointCloud::all.size() - 2], scan_node,
        wheel_diameter, wheel_separation, motor_cpr);
  }
  lidar_scan_postprocess(pc_new);

  // update_graph_boxes();
  AppState::get().graph_needs_update = true;
}

void car_follow_path(SOCKET &sockfd, std::vector<GridNode> &path,
                     GridGraph &navgraph, float wheel_diameter,
                     float wheel_separation, unsigned motor_cpr) {
  static glm::vec2 heading(-1, 0);
  const int rescan_interval = 20;
  DUMP(path.size());

  int ii = 0;
  if (path.size() != 0)
    for (auto i = path.begin(); i < path.end() - 1; i++) {
      if (ii == rescan_interval) {
        ii = 0;

        lidar_scan(sockfd, wheel_diameter, wheel_separation, motor_cpr,
                   AppState::get().car_position, AppState::get().car_angle);

        glm::vec3 start_world(AppState::get().car_position.x, 0,
                              AppState::get().car_position.y);
        glm::ivec2 start =
            world_to_grid(start_world, AppState::get().gridbox_size,
                          AppState::get().grid_offset);
        glm::ivec2 target = world_to_grid(AppState::get().target_world,
                                          AppState::get().gridbox_size,
                                          AppState::get().grid_offset);
        path = A_star(navgraph.graph, start.x, start.y, target.x, target.y);
        // i = path.begin();
        car_follow_path(sockfd, path, wheel_diameter, wheel_separation,
                        motor_cpr);
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
        status =
            car_turn(sockfd, 180, wheel_diameter, wheel_separation, motor_cpr);
      } else if (abs(cross_res - 1) < SMEPSILON) {
        xn_printf("left\n");
        status =
            car_turn(sockfd, 90, wheel_diameter, wheel_separation, motor_cpr);
      } else if (abs(cross_res + 1) < SMEPSILON) {
        xn_printf("right\n");
        status =
            car_turn(sockfd, -90, wheel_diameter, wheel_separation, motor_cpr);
      } else {
        xn_printf("ahead\n");
      }

      if (!status)
        return;

      int extra_square_count = 0;
      for (auto j = i + 2; j < (path.end()); j++) {
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

      status = car_move_forward(sockfd, 1.0f + extra_square_count,
                                AppState::get().gridbox_size, wheel_diameter,
                                motor_cpr);
      if (!status)
        return;
      time_sleep(3 + extra_square_count * 2);
      heading = heading_new;
      AppState::get().car_position +=
          heading * AppState::get().gridbox_size * (1.0f + extra_square_count);
    }

  if (heading == glm::vec2(0, 1))
    AppState::get().car_angle = (float)M_PI_2;
  else if (heading == glm::vec2(0, -1))
    AppState::get().car_angle = (float)-M_PI_2;
  else if (heading == glm::vec2(1, 0))
    AppState::get().car_angle = (float)M_PI;
  else if (heading == glm::vec2(-1, 0))
    AppState::get().car_angle = 0;
}

void scene_update(const gl::Camera &cam, std::vector<BoundedPoints> &boxes) {
  glm::vec3 def_scale{0.05, 0.05, 0.05};
  glm::vec3 car_scale = def_scale * 3.0f;

  gl::cls(0.2f, 0.3f, 0.3f);
  gl::begin_3d(cam, color_shader);
  gl::begin_3d(cam, lit_shader);
  lit_shader.setVec3("lightColor", 1.0f, 1.0f, 1.0f);
  lit_shader.setVec3("viewPos", cam.pos);
  lit_shader.setVec3("lightPos", glm::vec3(1, 10, 0));

  draw_map["car"]->setModelRecursive();
  glm::vec3 car_pos_3d(AppState::get().car_position.x, 0,
                       AppState::get().car_position.y);
  float car_ang_corrected = (float)-M_PI_2 + AppState::get().car_angle;
  draw_map["car"]->transform(car_pos_3d, car_scale, car_ang_corrected);

  draw_map["grid"]->setModelRecursive();
  draw_map["grid"]->transform(AppState::get().grid_offset -
                              glm::vec3(AppState::get().gridbox_size * 0.5, 0,
                                        AppState::get().gridbox_size * 0.5));

  for (auto i = 0; i < arm_verts.size(); i++) {
    arm_verts[i] = getArm()->positions[i].toGLM();
  }
  draw_map["arm"]->popChild(draw_map["arm_skeleton"]->id);
  draw_map["arm_skeleton"] = draw_map["arm"]->addChild(
      new DrawNode(&color_shader, new Mesh(arm_verts)));
  draw_map["arm_skeleton"]->meshes[0]->draw_mode = GL_LINE_STRIP;
  draw_map["arm_skeleton"]->addUniform("color",
                                       glm::vec4(1.0f, 1.0f, 0.2f, 1.0f));

  if (AppState::get().arm_target.mag_sqr() > SMEPSILON)
    draw_map["arm_target"]->visible = true;

  draw_map["arm"]->setModelRecursive();
  draw_map["arm"]->transform(car_pos_3d, glm::vec3(0.5), car_ang_corrected,
                             gl::axes.y);
  for (auto i = 0; i < getArm()->ideal_chain.bone_count; i++) {
    draw_map["arm"]->children[i]->transform(getArm()->positions[i].toGLM(),
                                            glm::vec3(0.1f));
  }

  const vec3 &t = AppState::get().arm_target;
  draw_map["arm_target"]->transform(glm::vec3(t.x, t.y, t.z), glm::vec3(0.1f));

  // draw pca obbs
  if (AppState::get().should_draw_boxes) {
    color_shader.setVec4("color", glm::vec4(1.0f, 1.0f, 0.0f, 1.0f));
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    for (BoundedPoints &bp : boxes)
      gl::draw_OBB(bp.box, color_shader);
  }

  if (XN_LIT) {
    gl::draw_path(boxmesh.VAO, lit_shader, AppState::get().gridgraph,
                  AppState::get().grid_offset, AppState::get().gridbox_size,
                  AppState::get().path);
  } else {
    gl::draw_path(boxmesh.VAO, color_shader, AppState::get().gridgraph,
                  AppState::get().grid_offset, AppState::get().gridbox_size,
                  AppState::get().path);
  }

  // add new navmeshes to scene
  if (draw_map["navmesh"]->children.size() < nav_verts.size()) {
    for (auto i = draw_map["navmesh"]->children.size(); i < nav_verts.size();
         i++) {
      DrawNode *n = draw_map["navmesh"]->addChild(
          new DrawNode(&color_shader, new Mesh(nav_verts[i])));
      n->addUniform("color", glm::vec4(0.8f, 0.6f, 0.4f, 0.5f));
    }
  }

  if (AppState::get().graph_needs_update) {
    update_graph_boxes();
    AppState::get().graph_needs_update = false;
  }

  if (AppState::get().scan_needs_update) {
    draw_map["scan"]->children.clear();
    for (auto &p : PointCloud::all) {
      DrawNode *c = draw_map["scan"]->addChild(
          new DrawNode(&color_shader, Mesh::gen_box_batch(p.points)));
      c->addUniform("color", gl::sample_colors[0]);
    }
    AppState::get().scan_needs_update = false;
  }
}

void wifi_car_test(SOCKET &sockfd, GridGraph &navgraph, float t = 1) {
  PointCloud p;
  for (int i = 0; i < 200; i++) {
    float ang = i * (float)M_PI * 2.0f / 200.0f;
    glm::vec3 pt = glm::vec3(AppState::get().car_position.x + cos(ang), 0,
                             AppState::get().car_position.y + sin(ang));
    p.points.push_back(pt);
    p.points.push_back(pt);
  }

  std::vector<glm::vec3> triverts;
  p.toNavmesh(triverts);
  nav_verts.push_back(triverts);
  PointCloud::all.clear();
  PointCloud::all.push_back(p);

  // convert to pathfinding grid
  AppState::get().gridgraph.clear();
  PointCloud::march_squares(PointCloud::all, 50, 50,
                            AppState::get().gridbox_size,
                            AppState::get().car_position, navgraph.cells,
                            AppState::get().grid_offset, 10, 2);

  navgraph.graph = preprocess_graph(navgraph.cells);
}
} // namespace xn