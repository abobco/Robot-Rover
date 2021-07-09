/*

    Car dims: 25.5cm x 18cm

    lidar distance from center: 11
*/

#pragma once
#include "../xn_gpio.hpp"
#include "../xn_math.hpp"
#include "../xn_net.hpp"
#include "../xn_pointcloud.hpp"
#include "../xn_vec.hpp"

#include "xn_button.hpp"
#include "xn_gl.hpp"
// #include "xn_gui.hpp"
#include "xn_renderer.hpp"

namespace xn {

const float WHEEL_DIAM = 9.4;
const float WHEEL_SEPARATION = 24;
const float MOT_CPR = 4320;

float car_angle = 0; // CCW rotation around +y axis
glm::vec2 car_position{0};
glm::vec3 target_world;

std::vector<GridNode> graph;
std::vector<GridNode> path;
std_vec2d<bool> gridgraph;
const float gridbox_size = 0.25;
glm::vec3 grid_offset;

DrawNode *graph_drawnode = NULL;
DrawNode *scan_drawnode = NULL;
DrawNode *grid_drawnode = NULL;
DrawNode *car_drawnode = NULL;
DrawNode *navmesh_drawnode = NULL;
DrawNode *axis_drawnode = NULL;

// std::vector<gl::VertexArrayInfo> navmesh;
std_vec2d<glm::vec3> nav_verts;
pthread_mutex_t mut;

Shader color_shader;
// DrawNode scene;
BoxMesh boxmesh;

int esp_sock;

struct {
    int scan_range_x = 100;
    int scan_range_y = 800;
    int cam_servo_width_x = 1500;
    int cam_servo_width_y = 1983;
    bool should_scan = false;
    bool ignore_keystrokes = false;
    bool should_restart_esp = false;
    bool window_settings_hovered = false;
    bool graph_needs_update = false;
    bool scan_needs_update = false;
} settings;

void reset_scene(DrawNode &scene) {
    scene = DrawNode();
    PointCloud::all.clear();
    nav_verts.clear();
    // navmesh.clear();
    graph.clear();
    gridgraph.clear();
}

void update_graph_boxes() {
    if (graph_drawnode == NULL) {
        graph_drawnode = DrawNode::scene.addChild(new DrawNode());
    }
    graph_drawnode->children.clear();

    std::vector<glm::vec3> graph_verts;
    for (unsigned i = 0; i < gridgraph.size(); i++)
        for (unsigned j = 0; j < gridgraph[i].size(); j++)
            if (!gridgraph[i][j]) {
                glm::vec3 v(i * gridbox_size, 0, j * gridbox_size);
                v += grid_offset;
                v.y = -gridbox_size * 0.5 - 0.01;
                graph_verts.push_back(v);
            }
    Mesh *m = Mesh::gen_box_batch(graph_verts, glm::vec3(gridbox_size));
    // m->setPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    DrawNode *g = graph_drawnode->addChild(new DrawNode(&color_shader, m));
    g->addUniform("color", glm::vec4(0.5f, 0.5f, 0.5f, 1.0f));
}

void gen_navgraph() {
    double s = glfwGetTime();
    PointCloud::march_squares(PointCloud::all, 50, 50, gridbox_size, car_position, gridgraph,
                              grid_offset, 10, 2);
    printf("marching squares took: %.2f s\n", glfwGetTime() - s);

    s = glfwGetTime();
    graph = preprocess_graph(gridgraph);
    pthread_mutex_unlock(&mut);
    printf("graph processing took: %.2f s\n", glfwGetTime() - s);
}

void load_scene() {
    axis_drawnode = DrawNode::scene.addChild(new DrawNode());
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
        DrawNode *n = axis_drawnode->addChild(new DrawNode(&color_shader, m));
        n->addUniform("color", col);
    }

    scan_drawnode = DrawNode::scene.addChild(new DrawNode());
    for (auto &p : PointCloud::all) {
        DrawNode *c =
            scan_drawnode->addChild(new DrawNode(&color_shader, Mesh::gen_box_batch(p.points)));
        c->addUniform("color", gl::sample_colors[0]);
    }

    // grid init
    grid_drawnode = DrawNode::scene.addChild(
        new DrawNode(&color_shader, Mesh::gen_grid(gridbox_size, 100, 100)));
    grid_drawnode->addUniform("color", glm::vec4(1.0f, 1.0f, 1.0f, 0.6f));
    grid_drawnode->transform(grid_offset - glm::vec3(gridbox_size * 0.5, 0, gridbox_size * 0.5));

    std::vector<float> triverts;
    triverts.insert(triverts.begin(), std::begin(gl::tri_verts_eq), std::end(gl::tri_verts_eq));
    Mesh *triangle = new Mesh(triverts);
    car_drawnode = DrawNode::scene.addChild(new DrawNode(&color_shader, triangle));
    car_drawnode->addUniform("color", glm::vec4(1.0f, 1.0f, 0.2f, 1.0f));

    // gen_navmesh();
    gen_navgraph();
    delete graph_drawnode;
    graph_drawnode = NULL;
    update_graph_boxes();

    navmesh_drawnode = DrawNode::scene.addChild(new DrawNode());

    for (auto &p : PointCloud::all) {
        nav_verts.push_back(std::vector<glm::vec3>());
        p.toNavmesh(nav_verts.back());
        DrawNode *n =
            navmesh_drawnode->addChild(new DrawNode(&color_shader, new Mesh(nav_verts.back())));
        n->addUniform("color", glm::vec4(0.8f, 0.6f, 0.4f, 0.5f));
    }
}

int car_move_forward(int &sockfd, float gridsquares, float gridbox_size) {
    float c = M_PI * WHEEL_DIAM;
    MotorInstruction cmd_m_forward;
    cmd_m_forward.length_pulses = (100 * gridsquares * gridbox_size) / (c / MOT_CPR);
    cmd_m_forward.dir_left = 0;
    cmd_m_forward.dir_right = 0;

    printf("sending motor instruction\n");
    // DUMP(cmd_m_forward.length_pulses);
    return send_pack(sockfd, cmd_m_forward);
}

int car_turn(int &sockfd, int deg = 90) {
    // deg / 360 = x / c
    float d = WHEEL_SEPARATION * M_PI * abs(deg) / 360;
    float c = M_PI * WHEEL_DIAM;
    // MOT_CPR / WHEEL_DIAM = x / c
    // x = c*MOT_CPR / 11
    MotorInstruction cmd_m_right;
    cmd_m_right.length_pulses = d / (c / MOT_CPR);
    if (deg > 0) {
        cmd_m_right.dir_left = 1;
        cmd_m_right.dir_right = 0;
    } else {
        cmd_m_right.dir_left = 0;
        cmd_m_right.dir_right = 1;
    }
    printf("sending motor instruction\n");
    int status = send_pack(sockfd, cmd_m_right);
    if (!status)
        return 0;
    time_sleep(2 * (float)abs(deg) / 90);
    car_angle += glm::radians((float)deg);
    return 1;
}

void lidar_scan_error_correct(int &sockfd, PointCloud &pc_new, const PointCloud &pc_prev,
                              DrawNode *scan_node, int iterations = 2) {
    printf("begin error correction: %.2f\n", glfwGetTime());
    for (int i = 0; i < iterations; i++) {
        glm::vec3 tx(0);
        float orig_ang = car_angle;
        double start_time = glfwGetTime();
        PointCloud::hillclimb_transform(pc_prev, pc_new, tx, car_angle);
        settings.scan_needs_update = true;
        DUMP(tx);
        DUMP(car_angle);

        printf("hillclimb %d took: %.2f s\n", i, glfwGetTime() - start_time);
        car_turn(sockfd, glm::degrees(orig_ang - car_angle));
        car_position.x += tx.x;
        car_position.y += tx.z;
    }
}

void lidar_scan_postprocess(const PointCloud &pc) {
    // fan triangulation
    double s = glfwGetTime();
    std::vector<glm::vec3> triverts;
    pc.toNavmesh(triverts);

    nav_verts.push_back(triverts);
    // printf("triangulation took: %.2f s\n", glfwGetTime() - s);

    // convert to pathfinding grid
    s = glfwGetTime();
    gridgraph.clear();
    PointCloud::march_squares(PointCloud::all, 50, 50, gridbox_size, car_position, gridgraph,
                              grid_offset, 10, 2);
    printf("marching squares took: %.2f s\n", glfwGetTime() - s);

    s = glfwGetTime();
    graph = preprocess_graph(gridgraph);
    printf("graph processing took: %.2f s\n", glfwGetTime() - s);
}

void lidar_scan_ex(PointCloud &pc_new, DrawNode *scan_node, int &sockfd, int min_width,
                   int max_width, int servo_step, int step_incr, int step_max,
                   const glm::vec2 &offset = glm::vec2(0, 0), float angle = 0) {
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
                    burst_size * sizeof(LidarStepInstruction) + 2 * burst_size * sizeof(uint32_t);
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
            // printf("ack1 ");
            acks += wait_ack(sockfd);
            // printf("ack2\n");
            if (acks < 2) {
                printf("resending remaining instructions\n");
                size_t bsent = i * (sizeof(LidarReadInstruction) + sizeof(LidarStepInstruction) +
                                    2 * sizeof(uint32_t));
                printf("%ld sent, %ld remaining\n", bsent, pack_size - bsent);
                send(sockfd, &bbuf[bsent], pack_size - bsent, 0);
                --i;
                continue;
            }
            read_int<uint16_t>(sockfd, d, false);

            step += step_incr;
            // DUMP(step);

            if (d == 65535)
                continue;
            float servo_ang_f = ((servo_ctl.width - min_width) / 2000.0f) * M_PI;
            double ang = -(step * (1.0 / 100) * M_PI);

            vec3 v = {(float)d * 0.01f, 0, 0};
            v = vec3::rotate_axis(v, {0, 1, 0}, ang + angle);
            vec3 rot_ax2 = vec3::rotate_axis({0, 0, 1}, {0, 1, 0}, ang);
            v = vec3::rotate_axis(v, rot_ax2, servo_ang_f);

            // DUMP(v);
            pthread_mutex_lock(&mut);
            pc_new.points.push_back(glm::vec3(v.x + offset.x, v.y, v.z + offset.y));
            DrawNode *c = new DrawNode(&color_shader, &boxmesh);
            c->transform(pc_new.points.back(), glm::vec3(0.05, 0.05, 0.05));
            c->addUniform("color", gl::sample_colors[0]);
            scan_node->addChild(c);
            pthread_mutex_unlock(&mut);
        }
    }
    printf("scan done\n");
}

void lidar_scan(int &sockfd, const glm::vec2 &offset = glm::vec2(0, 0), float angle = 0) {
    int min_width = 700, max_width = settings.scan_range_y, servo_step = 50;
    int step_incr = 1, step_max = settings.scan_range_x;

    PointCloud::all.push_back(PointCloud(glm::vec3(offset.x, 0, offset.y)));
    PointCloud &pc_new = PointCloud::all.back();

    DrawNode *scan_node = scan_drawnode->addChild(new DrawNode());

    lidar_scan_ex(pc_new, scan_node, sockfd, min_width, max_width, servo_step, step_incr, step_max,
                  offset, angle);

    car_turn(sockfd, 180);
    angle += M_PI;

    lidar_scan_ex(pc_new, scan_node, sockfd, min_width, max_width, servo_step, step_incr, step_max,
                  offset, angle);

    car_turn(sockfd, -180);

    if (PointCloud::all.size() > 1) {
        lidar_scan_error_correct(sockfd, pc_new, PointCloud::all[PointCloud::all.size() - 2],
                                 scan_node);
    }
    lidar_scan_postprocess(pc_new);

    // update_graph_boxes();
    settings.graph_needs_update = true;
}

void car_follow_path(int &sockfd, std::vector<GridNode> &path) {
    static glm::vec2 heading(-1, 0);
    const int rescan_interval = 4;
    DUMP(path.size());

    int ii = 0;
    if (path.size() != 0)
        for (auto i = path.begin(); i < path.end() - 1; i++) {
            if (ii == rescan_interval) {
                ii = 0;

                lidar_scan(sockfd, car_position, car_angle);

                glm::vec3 start_world(car_position.x, 0, car_position.y);
                glm::vec2 start = world_to_grid(start_world, gridbox_size, grid_offset);
                glm::vec2 target = world_to_grid(target_world, gridbox_size, grid_offset);
                path = A_star(graph, start.x, start.y, target.x, target.y);
                // i = path.begin();
                car_follow_path(sockfd, path);
                return;
            }

            GridNode &current = *i;
            GridNode &next = *(i + 1);
            glm::vec2 heading_new(next.x - current.x, next.y - current.y);
            float cross_res = glm::cross(glm::vec3(heading_new, 0), glm::vec3(heading, 0)).z;
            int status = 1;

            if (abs(glm::dot(heading_new, heading) + 1) < SMEPSILON) {
                printf("behind\n");
                status = car_turn(sockfd, 180);
            } else if (abs(cross_res - 1) < SMEPSILON) {
                printf("left\n");
                status = car_turn(sockfd, 90);
            } else if (abs(cross_res + 1) < SMEPSILON) {
                printf("right\n");
                status = car_turn(sockfd, -90);
            } else {
                status = printf("ahead\n");
            }

            if (!status)
                return;

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

            status = car_move_forward(sockfd, 1 + extra_square_count, gridbox_size);
            if (!status)
                return;
            time_sleep(3 + extra_square_count * 2);
            heading = heading_new;
            car_position += heading * gridbox_size * (1.0f + extra_square_count);
        }

    if (heading == glm::vec2(0, 1))
        car_angle = M_PI_2;
    else if (heading == glm::vec2(0, -1))
        car_angle = -M_PI_2;
    else if (heading == glm::vec2(1, 0))
        car_angle = M_PI;
    else if (heading == glm::vec2(-1, 0))
        car_angle = 0;
}

void wifi_car_test(int &sockfd, float t = 1) {
    PointCloud p;
    for (int i = 0; i < 200; i++) {
        float ang = i * M_PI * 2.0f / 200.0f;
        glm::vec3 pt = glm::vec3(car_position.x + cos(ang), 0, car_position.y + sin(ang));
        p.points.push_back(pt);
        p.points.push_back(pt);
    }

    std::vector<glm::vec3> triverts;
    p.toNavmesh(triverts);
    nav_verts.push_back(triverts);
    PointCloud::all.clear();
    PointCloud::all.push_back(p);

    // convert to pathfinding grid
    gridgraph.clear();
    PointCloud::march_squares(PointCloud::all, 50, 50, gridbox_size, car_position, gridgraph,
                              grid_offset, 10, 2);

    graph = preprocess_graph(gridgraph);
}
} // namespace xn