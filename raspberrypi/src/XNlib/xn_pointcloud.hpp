#pragma once
#include "xn_collisions.hpp"
#include "xn_math.hpp"
#include <array>
#include <fstream>
#include <utility>

#include <glm/gtx/norm.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <numeric>
namespace xn {

class PointCloud {
  public:
    static std::vector<PointCloud> all;
    glm::vec3 origin;
    std::vector<glm::vec3> points;

    PointCloud() {
        origin = glm::vec3(0);
        points = std::vector<glm::vec3>();
    }

    PointCloud(glm::vec3 origin, const std::vector<glm::vec3> &points = std::vector<glm::vec3>()) {
        this->origin = origin;
        this->points = points;
    }

    int appendToFile(std::ofstream &save_file, bool verbose = false) const {
        size_t total_size = (points.size() + 1) * sizeof(glm::vec3);
        DUMP(total_size);
        save_file.write((char *)&total_size, sizeof(size_t));
        save_file.write((char *)&origin, sizeof(glm::vec3));
        save_file.write((char *)&points.front(), points.size() * sizeof(glm::vec3));
        return total_size;
    }

    void toNavmesh(std::vector<glm::vec3> &out_tris) const {
        if (points.size() == 0)
            return;

        std::vector<glm::vec3> verts;
        sort_rays_by_angle(points, verts);

        // std::vector<glm::vec3> tri_verts;
        for (unsigned i = 0; i < verts.size() - 1; i++) {
            out_tris.push_back(origin);
            out_tris.push_back(verts[i]);
            out_tris.push_back(verts[i + 1]);
        }

        out_tris.push_back(origin);
        out_tris.push_back(verts.back());
        out_tris.push_back(verts.front());
    }

    static void save_pointclouds(std::string filename, const std::vector<PointCloud> &pcs = all) {
        std::ofstream save_file(filename, std::ofstream::binary);
        for (auto &p : pcs)
            p.appendToFile(save_file, true);
    }

    static void read_pointclouds(const std::string filename, std::vector<PointCloud> &pc_out) {
        size_t length, fpos = 0;
        std::ifstream indata(filename, std::ifstream::binary);

        indata.seekg(0, indata.end);
        length = indata.tellg();
        indata.seekg(0, indata.beg);
        DUMP(length);
        if (length > 0) {
            char *buf = new char[length];
            indata.read((char *)buf, length);
            while (fpos < length - 1) {
                size_t *sbuf = (size_t *)&buf[fpos];
                size_t cur_len = sbuf[0];
                DUMP(cur_len);
                glm::vec3 *vbuf = (glm::vec3 *)&sbuf[1];

                glm::vec3 orig = vbuf[0];
                std::vector<glm::vec3> points;
                points.insert(points.begin(), &vbuf[1], &vbuf[1] + cur_len / sizeof(glm::vec3) - 1);
                pc_out.push_back(PointCloud(orig, points));
                fpos += cur_len + sizeof(size_t);
            }
        }
    }

    static void combine_points(const std::vector<PointCloud> &inp, std::vector<glm::vec3> &out) {
        std::vector<glm::vec3> o;

        for (const PointCloud &p : inp) {
            o.insert(o.end(), p.points.begin(), p.points.end());
        }
        out = o;
    }

    static void join(const PointCloud &a, const PointCloud &b, PointCloud &out) {
        std::vector<glm::vec3> verts;
        verts.insert(verts.begin(), a.points.begin(), a.points.end());
        verts.insert(verts.begin(), b.points.begin(), b.points.end());
        // float tot = std::accumulate(verts.begin(), verts.end(), 0.0f);
        // tot /= verts.size();
        glm::vec3 c = a.origin + b.origin;
        c *= 0.5;
        out = PointCloud(c, verts);
    }

    static void hillclimb_transform(const PointCloud &a, PointCloud &b, glm::vec3 &t, float &angle,
                                    float ang_step = 0.01) {

        std::vector<glm::vec3> a_proj, b_proj, n_b;
        project_points(a.points, a_proj, {0, 0, 0}, {0, 1, 0});
        project_points(b.points, b_proj, {0, 0, 0}, {0, 1, 0});

        auto get_error = [](std::vector<glm::vec3> &avec, std::vector<glm::vec3> &bvec) -> float {
            std::vector<float> min_distances;
            for (glm::vec3 &p_a : avec) {
                float min_dist = FLT_MAX;
                for (glm::vec3 &p_b : bvec) {
                    float d = glm::distance2(p_a, p_b);
                    if (d < min_dist) {
                        min_dist = d;
                    }
                }
                min_distances.push_back(min_dist);
            }
            float sm = std::accumulate(min_distances.begin(), min_distances.end(), 0.0f);
            sm /= min_distances.size();
            return sm;
        };
        float orig_error = get_error(a_proj, b_proj);
        // DUMP(orig_error);

        auto rotate = [](glm::vec3 &a, float x) { a = glm::rotate(a, x, {0, 1, 0}); };
        auto translate_x = [](glm::vec3 &a, float x) { a.x += x; };
        auto translate_z = [](glm::vec3 &a, float x) { a.z += x; };
        auto transform_wrapper = [&](std::vector<glm::vec3> &vec, std::vector<glm::vec3> &scratch,
                                     float &accum, float incr,
                                     void (*transform)(glm::vec3 &, float)) {
            while (1) {
                scratch = vec;
                for (glm::vec3 &p : scratch) {
                    p -= b.origin;
                    transform(p, incr);
                    p += b.origin;
                }
                accum += incr;
                float new_error = get_error(a_proj, scratch);
                if (new_error > orig_error) {
                    scratch = vec;
                    break;
                }
                vec = scratch;
                orig_error = new_error;
            }
        };

        transform_wrapper(b_proj, n_b, angle, ang_step, rotate);
        transform_wrapper(b_proj, n_b, angle, -ang_step, rotate);
        transform_wrapper(b_proj, n_b, t.x, ang_step, translate_x);
        transform_wrapper(b_proj, n_b, t.x, -ang_step, translate_x);
        transform_wrapper(b_proj, n_b, t.z, ang_step, translate_z);
        transform_wrapper(b_proj, n_b, t.z, -ang_step, translate_z);
        for (unsigned i = 0; i < b.points.size(); i++) {
            b.points[i].x = n_b[i].x;
            b.points[i].z = n_b[i].z;
        }

        // DUMP(get_error(a_proj, n_b));
        // DUMP(angle);
        // DUMP(t);
    }

    static void march_squares(const std::vector<PointCloud> &pcs, const int w, const int h,
                              const float s, const glm::vec2 &car_position,
                              std_vec2d<bool> &grid_out, glm::vec3 &offset_out,
                              const float distance_cutoff = 10,
                              unsigned points_per_square_threshold = 1) {
        points_per_square_threshold *= pcs.size() / 3;
        points_per_square_threshold++;
        std::vector<glm::vec3> all_points;
        combine_points(pcs, all_points);
        project_points(all_points, all_points, {0, 0, 0}, {0, 1, 0});
        if (all_points.size()) {
            offset_out = glm::vec3(FLT_MAX);
        } else {
            offset_out = glm::vec3(0);
        }
        for (auto p : all_points) {
            for (int i = 0; i < 3; i++)
                if (offset_out[i] > p[i] && abs(p[i]) < distance_cutoff)
                    offset_out[i] = p[i];
        }

        for (glm::vec3 &p : all_points)
            p -= offset_out;

        // auto start = pio::get_time();
        std::vector<glm::vec2> navmesh;
        for (const PointCloud &p : pcs) {
            std::vector<glm::vec3> m3;
            std::vector<glm::vec2> m2;
            p.toNavmesh(m3);
            for (glm::vec3 &m : m3)
                m2.push_back({m.x - offset_out.x, m.z - offset_out.z});

            navmesh.insert(navmesh.end(), m2.begin(), m2.end());
        }
        // printf("navmesh gen took %.2f s\n", pio::time_diff_seconds(start, pio::get_time()));

        glm::vec3 cp(car_position.x - offset_out.x, 0, car_position.y - offset_out.z);
        offset_out += 0.5 * s;
        offset_out.y = 0;

        grid_out.clear();
        float time_collision_tests = 0, time_spatial_hashing = 0;
        for (int i = 0; i < w; i++) {
            auto sp_start = pio::get_time();
            grid_out.push_back(std::vector<bool>());
            for (int j = 0; j < h; j++) {
                if (spatial_hash(&cp, 1, i, 0, j, s).size() == 1) {
                    grid_out[i].push_back(1);
                    time_spatial_hashing += pio::time_diff_seconds(sp_start, pio::get_time());
                } else if (spatial_hash(&all_points.front(), all_points.size(), i, 0, j, s)
                               .size() >= points_per_square_threshold) {
                    grid_out[i].push_back(0);
                    time_spatial_hashing += pio::time_diff_seconds(sp_start, pio::get_time());
                } else {
                    std::vector<glm::vec2> box({{i, j}, {i + 1, j}, {i + 1, j + 1}, {i, j + 1}});
                    for (auto &v : box) {
                        v *= s;
                    }
                    bool known_square = false;
                    for (unsigned j = 0; j < navmesh.size(); j += 3) {
                        std::vector<glm::vec2> tri;
                        tri.push_back(navmesh[j]);
                        tri.push_back(navmesh[j + 1]);
                        tri.push_back(navmesh[j + 2]);

                        auto s = pio::get_time();
                        if (collision_convex_polygons(tri, box)) {
                            grid_out[i].push_back(1);
                            known_square = true;
                            break;
                        }
                        time_collision_tests += pio::time_diff_seconds(s, pio::get_time());
                    }
                    if (!known_square)
                        grid_out[i].push_back(0);
                }
            }
        }
        // DUMP(time_collision_tests);
        // DUMP(time_spatial_hashing);
    }
};
} // namespace xn