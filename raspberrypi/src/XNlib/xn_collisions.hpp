#include "xn_math.hpp"

namespace xn {

struct Plane {
    float d; // distance along norm from origin
    glm::vec3 norm;
};

int test_separating_axis(const std::vector<glm::vec2> &a, const std::vector<glm::vec2> &b,
                         const glm::vec2 &axis) {
    int res = 1;

    ExtremeInfo r[2];
    extreme_pts_in_dir(axis, &a.front(), a.size(), r[0]);
    extreme_pts_in_dir(axis, &b.front(), b.size(), r[1]);

    if (r[1].proj_min > r[0].proj_max || r[1].proj_max < r[0].proj_min) {
        res = 0;
    }
    return res;
}

int collision_convex_polygons(const std::vector<glm::vec2> &a, const std::vector<glm::vec2> &b) {
    // test against t's axes
    auto f = [&](const std::vector<glm::vec2> &t) -> int {
        for (unsigned i = 0; i < t.size(); i++) {
            int next_id = (i + 1) % 3;
            glm::vec2 axis = {-(t[next_id].y - t[i].y), t[next_id].x - t[i].x};
            axis = glm::normalize(axis);
            if (!test_separating_axis(a, b, axis))
                return 0;
        }
        return 1;
    };
    if (!f(a) || !f(b))
        return 0;
    return 1;
}

int collision_segment_plane(const glm::vec3 &a, const glm::vec3 &b, const Plane &p, float &t,
                            glm::vec3 &q) {
    glm::vec3 ab = b - a;
    t = (p.d - glm::dot(p.norm, a)) / glm::dot(p.norm, ab);

    if (t >= 0.0f && t <= 1.0f) {
        q = a + ab * t;
        return 1;
    }

    return 0;
}
} // namespace xn