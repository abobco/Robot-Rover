#pragma once

#include <algorithm>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

// #include "opengl/xn_gl.hpp"
#include "xn_vec.hpp"

namespace xn {
template <class T> using std_vec2d = std::vector<std::vector<T>>;

struct Eigen {
    float val;
    glm::vec3 vec;
};

// print glm types w/ iostream
std::ostream &operator<<(std::ostream &os, const glm::vec2 &r) {
    return os << "(" << r.x << ", " << r.y << ", "
              << ")";
}

std::ostream &operator<<(std::ostream &os, const glm::vec3 &r) {
    return os << "(" << r.x << ", " << r.y << ", " << r.z << ")";
}
std::ostream &operator<<(std::ostream &os, const glm::mat3 &r) {
    os << "\n";
    for (int i = 0; i < 3; i++) {
        os << "[ ";
        for (int j = 0; j < 3; j++)
            os << r[i][j] << " ";
        os << "]\n";
    }
    return os;
}

template <class T> std::vector<T> flatten_array(std_vec2d<T> &a) {
    std::vector<T> out;
    for (auto &s : a)
        for (T &v : s)
            out.push_back(v);
    return out;
}

float expected_val(float v[], int num_points, float p[] = NULL) {
    float eval = 0;
    if (p == NULL) {
        float prob = 1.0f / num_points;
        for (int i = 0; i < num_points; i++) {
            eval += v[i] * prob;
        }
    } else {
        for (int i = 0; i < num_points; i++) {
            eval += v[i] * p[i];
        }
    }
    return eval;
}

float stdev_3d(const std::vector<glm::vec3> &samples) {
    glm::vec3 d(0);

    glm::vec3 cen;
    for (auto v : samples)
        cen += v / (float)samples.size();

    for (auto v : samples)
        d = (v - cen) * (v - cen);

    d /= (float)samples.size();

    return glm::length(d);
}

float covariance(float v1[], float v2[], int num_vars) {
    float ex1 = expected_val(v1, num_vars);
    float ex2 = expected_val(v2, num_vars);

    float vn1[num_vars];
    float vn2[num_vars];
    for (int i = 0; i < num_vars; i++) {
        vn1[i] = v1[i] - ex1;
        vn2[i] = v2[i] - ex2;
    }

    for (int i = 0; i < num_vars; i++) {
        vn1[i] = vn1[i] * vn2[i];
    }
    return expected_val(vn1, num_vars);
}

float **cov_mat(float **df, int rows, int cols, bool fill_redundant_values = true) {
    float **cov_mat = new float *[cols];
    for (int i = 0; i < cols; i++) {
        cov_mat[i] = new float[cols];
    }
    for (int i = 0; i < cols; i++) {
        for (int j = 0; j < i + 1; j++) {
            cov_mat[i][j] = covariance(df[i], df[j], rows);
        }
    }

    if (fill_redundant_values)
        for (int i = 0; i < cols; i++)
            for (int j = i + 1; j < cols; j++)
                cov_mat[i][j] = cov_mat[j][i];

    return cov_mat;
}

float trace(float **a, int n) {
    float total = 0;
    for (int i = 0; i < n; i++)
        total += a[i][i];
    return total;
}

float trace(glm::mat3 a) {
    float total = 0;
    for (int i = 0; i < 3; i++)
        total += a[i][i];
    return total;
}

glm::mat3 raw_mat_to_glm3x3(float **a) {
    float a_flat[9];
    for (int i = 0; i < 9; i++) {
        a_flat[i] = a[i / 3][i % 3];
    }
    return glm::make_mat3(a_flat);
}

// get eigenvalues for a symmetric, real, 3x3 matrix. 1 of the values may be
// repeated
glm::vec3 eigenval_3x3(glm::mat3 a) {
    glm::vec3 eigv;
    float p1 = a[0][1] * a[0][1] + a[0][2] * a[0][2] + a[1][2] * a[1][2];
    if (p1 == 0) {
        // matrix is diagonal => eigenvalues are the 3 diagonal values
        for (int i = 0; i < 3; i++) {
            eigv[i] = a[i][i];
        }
    } else {
        float q = trace(a) / 3;
        float p2 = (a[0][0] - q) * (a[0][0] - q) + (a[1][1] - q) * (a[1][1] - q) +
                   (a[2][2] - q) * (a[2][2] - q) + 2 * p1;
        float p = sqrt(p2 / 6);
        glm::mat3 B(0);
        glm::mat3 qi(q);
        B = (1 / p) * (a - qi);
        float r = glm::determinant(B) / 2;

        // matrix must be symmetric w/ -1 <= r <= 1
        // computation error can leave r slightly outside this range
        float phi;
        if (r <= -1)
            phi = M_PI / 3;
        else if (r >= 1)
            phi = 0;
        else
            phi = acos(r) / 3;

        // eigenvalues satisfy eig[2] <= eig[1] <= eig[0]
        eigv[0] = q + 2 * p * cos(phi);
        eigv[2] = q + 2 * p * cos(phi + 2 * M_PI / 3);
        eigv[1] = 3 * q - eigv[0] - eigv[2]; // b/c trace(a) = eig[0] + eig[1] + eig[2]
    }

    return eigv;
}

glm::vec3 find_eigvec(glm::mat3 &eigmat, bool print_info = false) {
    glm::vec3 out(0);
    for (int j = 0; j < 3; j++)
        for (int k = 0; k < 3; k++)
            if (abs(eigmat[j][k]) > SMEPSILON) {
                return eigmat[j];
            }
    if (print_info)
        std::cout << "eigmat was all zeros!\n";
    return out;
}

std::vector<Eigen> eigenvec_3x3(float **a) {
    glm::mat3 ga = raw_mat_to_glm3x3(a);
    glm::vec3 eigval = eigenval_3x3(ga);
    std::vector<Eigen> out;

    bool mult = false;
    float multval = 0, othval = 0;
    for (int i = 0; i < 3; i++)
        for (int j = i + 1; j < 3; j++)
            if (eigval[i] == eigval[j]) {
                mult = true;
                multval = eigval[i];
            }

    if (mult) {
        // 2 eigenvalues, 3rd vector will be filled in w/ cross product
        glm::mat3 I1(multval);
        glm::mat3 I2(othval);
        glm::mat3 eigmat[2];
        glm::vec3 eigvec[2];
        for (int i = 0; i < 3; i++)
            if (eigval[i] != multval)
                othval = eigval[i];
        eigmat[0] = (ga - I1) * (ga - I1);
        eigmat[1] = (ga - I2) * (ga - I1);

        eigvec[0] = find_eigvec(eigmat[0]);
        eigvec[1] = find_eigvec(eigmat[1]);

        out.push_back({eigval[0], eigvec[0]});
        out.push_back({eigval[1], eigvec[1]});
        out.push_back({1, glm::cross(eigvec[0], eigvec[1])}); // get 3rd vector thats perpendicular
                                                              // to the 2 real eigenvectors
    } else {
        // 3 eigenvalues: exact solution
        glm::mat3 I0(eigval[0]);
        glm::mat3 I1(eigval[1]);
        glm::mat3 I2(eigval[2]);
        glm::mat3 eigmat[3];
        eigmat[0] = (ga - I1) * (ga - I2);
        eigmat[1] = (ga - I0) * (ga - I2);
        eigmat[2] = (ga - I0) * (ga - I1);

        for (int i = 0; i < 3; i++)
            eigmat[i] = glm::transpose(eigmat[i]);

        for (int i = 0; i < 3; i++) {
            glm::vec3 e = find_eigvec(eigmat[i]);
            if (e != glm::vec3(0))
                out.push_back({eigval[i], e});
        }
    }
    return out;
}

struct ExtremeInfo {
    int idx_min;
    int idx_max;
    float proj_min;
    float proj_max;
};

template <class T>
void extreme_pts_in_dir(const T &dir, const T pt[], int n, ExtremeInfo &out, float threshold = 10) {
    out.proj_min = FLT_MAX;
    out.proj_max = -FLT_MAX;
    for (int i = 0; i < n; i++) {
        float proj = glm::dot(pt[i], dir);
        if (abs(proj) > threshold)
            continue;

        if (proj < out.proj_min) {
            out.proj_min = proj;
            out.idx_min = i;
        }
        if (proj > out.proj_max) {
            out.proj_max = proj;
            out.idx_max = i;
        }
    }
}

std::vector<glm::vec3> spatial_hash(glm::vec3 pt[], int n, int x, int y, int z, float cube_size) {
    std::vector<glm::vec3> out;
    glm::vec3 bmin(x, y, z);
    bmin *= cube_size;
    glm::vec3 bmax(bmin);
    bmax += cube_size;

    // DUMP(bmin);
    // DUMP(bmax);

    for (int i = 0; i < n; i++) {
        if (pt[i].x >= bmin.x && pt[i].x <= bmax.x && pt[i].y >= bmin.y && pt[i].y <= bmax.y &&
            pt[i].z >= bmin.z && pt[i].z <= bmax.z)
            out.push_back(pt[i]);
    }

    return out;
}

struct OBB {
    glm::vec3 pos;
    glm::vec3 ext[3];
};

bool eigenComp(Eigen a, Eigen b) { return a.val < b.val; }

OBB pca_obb(std::vector<glm::vec3> pt, bool print_info = false) {
    OBB out;
    out.pos = glm::vec3(0);
    unsigned int i, j;
    for (i = 0; i < 3; i++)
        out.ext[i] = glm::vec3(0);
    float **df = new float *[3];
    for (i = 0; i < 3; i++) {
        df[i] = new float[pt.size()];
    }
    // data must be a float **
    for (i = 0; i < 3; i++)
        for (j = 0; j < pt.size(); j++)
            df[i][j] = pt[j][i];

    // subtract means
    for (i = 0; i < 3; i++) {
        float m = expected_val(df[i], pt.size());
        for (j = 0; j < pt.size(); j++)
            df[i][j] -= m;
    }

    // covariance matrix
    float **cm = cov_mat(df, pt.size(), 3);
    for (i = 0; i < 3; i++)
        delete[] df[i];
    delete[] df;

    // eigenvectors => rotation axes of box
    std::vector<Eigen> eigenvectors = eigenvec_3x3(cm);
    for (i = 0; i < 3; i++)
        delete[] cm[i];
    delete[] cm;

    std::sort(eigenvectors.begin(), eigenvectors.end(), eigenComp);
    if (print_info) {
        printf("eigenvectors:\n");
        for (Eigen &e : eigenvectors)
            printf("%f : (%.3f, %.3f, %.3f)\n", e.val, e.vec.x, e.vec.y, e.vec.z);
    }
    for (Eigen &e : eigenvectors)
        e.vec = glm::normalize(e.vec);

    // create bounding box
    glm::vec3 centroid(0);
    glm::vec3 extents[6];
    glm::vec3 corners[8];
    for (i = 0; i < pt.size(); i++)
        centroid += (pt[i] / (float)pt.size());

    for (i = 0; i < pt.size(); i++)
        pt[i] -= centroid;
    for (i = 0; i < 3; i++) {
        ExtremeInfo extrema;
        extreme_pts_in_dir(eigenvectors[i].vec, &pt.front(), pt.size(), extrema);
        extents[i * 2] = eigenvectors[i].vec * extrema.proj_max;
        extents[i * 2 + 1] = eigenvectors[i].vec * extrema.proj_min;
    }
    for (i = 0; i < pt.size(); i++)
        pt[i] += centroid;

    float half_len11 = glm::length(extents[2]);
    float half_len12 = glm::length(extents[3]);
    float half_len21 = glm::length(extents[4]);
    float half_len22 = glm::length(extents[5]);
    glm::vec3 ax1 = glm::normalize(extents[2]), ax2 = glm::normalize(extents[4]);

    corners[0] = extents[0] + ax1 * half_len11 + ax2 * half_len21;
    corners[1] = extents[0] - ax1 * half_len12 + ax2 * half_len21;
    corners[2] = extents[0] + ax1 * half_len11 - ax2 * half_len22;
    corners[3] = extents[0] - ax1 * half_len12 - ax2 * half_len22;

    corners[4] = extents[1] + ax1 * half_len11 + ax2 * half_len21;
    corners[5] = extents[1] - ax1 * half_len12 + ax2 * half_len21;
    corners[6] = extents[1] + ax1 * half_len11 - ax2 * half_len22;
    corners[7] = extents[1] - ax1 * half_len12 - ax2 * half_len22;

    for (i = 0; i < 8; i++) {
        glm::vec3 fac = (corners[i] + centroid);
        fac *= 1.0 / 8;
        out.pos += fac;
    }
    if (print_info)
        printf("box_cen: %f,%f,%f\n", out.pos.x, out.pos.y, out.pos.z);

    for (i = 0; i < 3; i++) {
        out.ext[i] = glm::normalize(extents[i * 2]);
        float d = glm::length(extents[i * 2]) + glm::length(extents[i * 2 + 1]);
        out.ext[i] *= d;
    }
    return out;
}

// gets closest points c1 & c2 of S1(s) = P1 + s*(Q1-P1) and S2(t) = P2 +
// t*(Q2-P2), returning s & t. function result is squared distance between S1(s)
// and S2(t)
float closest_pt_segment_segment(glm::vec3 p1, glm::vec3 q1, glm::vec3 p2, glm::vec3 q2, float &s,
                                 float &t, glm::vec3 &c1, glm::vec3 &c2) {
    glm::vec3 d1 = q1 - p1;
    glm::vec3 d2 = q2 - p2;
    glm::vec3 r = p1 - p2;
    float a = glm::dot(d1, d1);
    float e = glm::dot(d2, d2);
    float f = glm::dot(d2, r);

    if (a < SMEPSILON && e < SMEPSILON) {
        // both segments degenerate to points
        s = t = 0.0f;
        c1 = p1;
        c2 = p2;
        return glm::dot(c1 - c2, c1 - c2);
    }
    if (a <= SMEPSILON) {
        s = 0.0f;
        t = f / e; // s = 0 => t = (b*s + f)/e = f/e
        t = clamp<float>(t, 0.0f, 1.0f);
    } else {
        float c = glm::dot(d1, r);
        if (e <= SMEPSILON) {
            // 2nd segment degenerates to a point
            s = clamp<float>(-c / a, 0.0f,
                             1.0f); // t = 0 => s = (b*t - c)/a = -c/a
        } else {
            // general case
            float b = glm::dot(d1, d2);
            float denom = a * e - b * b; // always non-negative

            // if segments not parallel, compute closest point on L1 to L2 and
            // campl to segent s1
            if (denom != 0.0f) {
                s = clamp<float>((b * f - c * e) / denom, 0.0f, 1.0f);
            } else
                s = 0.0f; // if not parallel, pick arbitrary s

            // get point on L2 closest to S1(s) using
            // t = dot( (p1 + d1*s)-P2, d2) / dot(d2,d2) = (b*s + f) / e
            t = (b * s + f) / e;

            // if t in [0,1] done. Else clamp t, recompute s for ne value of t,
            // clamp s to [0,1]
            if (t < 0.0f) {
                t = 0.0f;
                s = clamp(-c / a, 0.0f, 1.0f);
            } else if (t > 1.0f) {
                t = 1.0f;
                s = clamp<float>((b - c) / a, 0.0f, 1.0f);
            }
        }
    }

    c1 = p1 + d1 * s;
    c2 = p2 + d2 * t;
    return glm::dot(c1 - c2, c1 - c2);
}

void project_points(const std::vector<glm::vec3> &points_in, std::vector<glm::vec3> &points_out,
                    glm::vec3 plane_p, glm::vec3 plane_norm) {
    std::vector<glm::vec3> out;
    for (auto p : points_in) {
        glm::vec3 d = p - plane_p;
        glm::vec3 proj = plane_norm * glm::dot(d, plane_norm);
        out.push_back(p - proj);
    }
    points_out = out;
}

struct VectorSortInfo {
    glm::vec3 vec;
    float dot;
};

bool dotComp(VectorSortInfo a, VectorSortInfo b) { return a.dot < b.dot; }

void sort_rays_by_angle(const std::vector<glm::vec3> &in_vec, std::vector<glm::vec3> &out_vec) {
    std::vector<glm::vec3> out;

    // project onto ground plane
    project_points(in_vec, out, glm::vec3(0), glm::vec3(0, 1, 0));

    // sort lines by angle from origin
    std::vector<VectorSortInfo> vecs;
    for (auto v : out) {
        glm::vec3 nv = glm::normalize(v);
        float t = glm::dot(nv, glm::vec3(0, 0, 1));
        if (glm::cross(nv, glm::vec3(0, 0, 1)).y < 0)
            t = -2 - t;
        vecs.push_back({v, t});
    }

    std::sort(vecs.begin(), vecs.end(), dotComp);
    out.clear();

    for (auto vd : vecs)
        out.push_back(vd.vec);

    out_vec = out;
}

void scan_split_lines(const std_vec2d<glm::vec3> &samples, std_vec2d<glm::vec3> &samples_out) {
    std_vec2d<glm::vec3> out;
    // for ( auto s : samples ) {

    //     // project onto ground plane
    //     out.push_back(std::vector<glm::vec3>());
    //     project_points(s, out.back(), glm::vec3(0), glm::vec3(0,1,0));

    //     // sort lines by angle from origin
    //     std::vector<VectorSortInfo> vecs;
    //     for (auto v : out.back()) {
    //         glm::vec3 nv = glm::normalize(v);
    //         float t = glm::dot(nv, glm::vec3(0,0,1));
    //         if ( glm::cross(nv, glm::vec3(0,0,1)).y < 0 )
    //             t = -2-t;
    //         vecs.push_back({ v, t});
    //     }

    //     std::sort( vecs.begin(), vecs.end(), dotComp );
    //     out.back().clear();

    //     for ( auto vd : vecs )
    //         out.back().push_back(vd.vec);
    // }
    // samples_out = out;
    for (const std::vector<glm::vec3> &s : samples) {
        glm::vec3 v_prev(0);
        float slope_prev = 0;
        bool prev_streak = false;
        for (const glm::vec3 &v : s) {
            glm::vec3 d = v - v_prev;
            float slope = d.z / d.x;
            if (prev_streak) {
                slope_prev = 0;
                for (unsigned int i = 1; i < samples_out.back().size(); i++) {
                    d = samples_out.back()[i] - samples_out.back()[i - 1];
                    slope_prev += ((d.z / d.x) / (float)(samples_out.back().size() - 1));
                }
            }

            bool connected = false;
            if (abs(slope - slope_prev) < 0.1)
                connected = true;

            if (connected) {
                if (!prev_streak) {
                    samples_out.push_back(std::vector<glm::vec3>());
                    samples_out.back().push_back(v_prev);
                }

                samples_out.back().push_back(v);
                v_prev = samples_out.back().front();
            } else

                v_prev = v;

            prev_streak = connected;
            slope_prev = slope;
        }
    }

    // combine intersecting lines
    std_vec2d<glm::vec3>::iterator its;
    std_vec2d<glm::vec3>::iterator its2;
    for (its = samples_out.begin(); its < samples_out.end(); its++)
        if (its->size() < 4)
            samples_out.erase(its);

    for (its = samples_out.begin(); its < samples_out.end(); its++) {
        for (its2 = its + 1; its2 < samples_out.end(); its2++) {
            glm::vec3 c1, c2;
            float s, t, distance_sqr;

            glm::vec3 d1 = its->back() - its->front();
            glm::vec3 d2 = its2->back() - its2->front();

            float s1 = d1.z / d1.x;
            float s2 = d2.z / d2.x;

            distance_sqr = closest_pt_segment_segment(its->front(), its->back(), its2->front(),
                                                      its2->back(), s, t, c1, c2);

            if (distance_sqr < 0.05f && abs(s1 - s2) < 0.1) {
                // std::vector<glm::vec3> newvec;
                // newvec.insert(newvec.end(), its->begin(), its->end());
                // newvec.insert(newvec.end(), its2->begin(), its2->end());

                DUMP(distance_sqr);
                // samples_out.erase(its);
                its->insert(its->end(), its2->begin(), its2->end());
                samples_out.erase(its2);
            }
        }
    }
}
} // namespace xn