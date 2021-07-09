/*
    opengl 3.1, glfw, and glad helper functions
*/
#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "shader.h"

#include "glad/glad.h"
#include <GLFW/glfw3.h>

#include "../robo/xn_search.hpp"
// #include "xn_text.hpp"
#include "xn_texture.hpp"
#include "xn_vertex_arrays.hpp"
#include <array>
#include <assert.h>
#include <filesystem>
#include <utility>

template <class T, class... Tail, class Elem = typename std::decay<T>::type>
std::array<Elem, 1 + sizeof...(Tail)> make_array(T &&head, Tail &&...values) {
  return {std::forward<T>(head), std::forward<Tail>(values)...};
}

namespace xn {
namespace gl {

struct VertexArrayInfo {
  unsigned int VAO;
  unsigned int VBO;
  unsigned int stride;
  size_t length;
  float *array;
};

static const struct {
  glm::vec3 x{1, 0, 0};
  glm::vec3 y{0, 1, 0};
  glm::vec3 z{0, 0, 1};
} axes;

static const auto sample_colors = make_array(glm::vec4{1.0f, 0.5f, 0.2f, 1.0f},
                                             glm::vec4{0.0f, 1.0f, 0.2f, 1.0f},
                                             glm::vec4{0.0f, 0.5f, 1.0f, 1.0f});

static const glm::vec3 origin(0);

static glm::ivec2 win_size_pixels(0);

GLFWwindow *CreateWindow_glfw(int width, int height, std::string name) {
  glfwInit();
  // std::vector<GLFWmonitor *> monitors;
  // {
  //   int count;
  //   GLFWmonitor **mt = glfwGetMonitors(&count);
  //   for (int i = 0; i < count; i++) {
  //     monitors.push_back(mt[i]);
  //   }
  // }
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
  GLFWwindow *window =
      glfwCreateWindow(width, height, name.c_str(), NULL, NULL);
  if (window == NULL) {
    std::cout << "Failed to create GLFW window" << std::endl;
    glfwTerminate();
    throw -1;
  }
  glfwMakeContextCurrent(window);
  win_size_pixels = glm::ivec2(width, height);

  // glad: load all OpenGL function pointers
  // ---------------------------------------
  if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
    std::cout << "Failed to initialize GLAD" << std::endl;
    throw -1;
  }

  return window;
}

struct Window {
  GLFWwindow *handle;
  Window(int width, int height, std::string name) {
    handle = CreateWindow_glfw(width, height, name);
  }

  glm::ivec2 getSize() {
    glm::ivec2 s;
    glfwGetWindowSize(handle, &s.x, &s.y);
    return s;
  }

  void resize(int x, int y) {
    gl::win_size_pixels = {x, y};
    glfwSetWindowSize(handle, x, y);
    glViewport(0, 0, x, y);
  }

  void close() { glfwSetWindowShouldClose(handle, true); }
  bool shouldClose() { return glfwWindowShouldClose(handle); }

  glm::dvec2 getCursorPos() {
    glm::dvec2 c;
    glfwGetCursorPos(handle, &c.x, &c.y);
    return c;
  }

  int getKey(int key) { return glfwGetKey(handle, key); }

  void flip() { glfwSwapBuffers(handle); }
};

struct RenderTexture {
  GLuint fb_handle = 0;
  GLuint tex_handle = 0;
  GLuint depthbuffer_handle = 0;
  GLuint rbo;
  GLuint quadVAO, quadVBO;
  glm::ivec2 dimensions = glm::ivec2(124);

  float quadVertices[24] =
      { // vertex attributes for a quad that fills the entire
        // screen in Normalized Device Coordinates. NOTE that
        // this plane is now much smaller and at the top of
          -0.3f, 1.0f, 0.0f, 1.0f, -0.3f, 0.7f, 0.0f, 0.0f,
          0.3f,  0.7f, 1.0f, 0.0f, -0.3f, 1.0f, 0.0f, 1.0f,
          0.3f,  0.7f, 1.0f, 0.0f, 0.3f,  1.0f, 1.0f, 1.0f};

  GLenum DrawBuffers[1] = {GL_COLOR_ATTACHMENT0};

  RenderTexture(){};

  RenderTexture(int w, int h) {
    dimensions.x = w;
    dimensions.y = h;

    // screen quad VAO
    glGenVertexArrays(1, &quadVAO);
    glGenBuffers(1, &quadVBO);
    glBindVertexArray(quadVAO);
    glBindBuffer(GL_ARRAY_BUFFER, quadVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(quadVertices), &quadVertices,
                 GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float),
                          (void *)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float),
                          (void *)(2 * sizeof(float)));

    // create framebuffer
    glGenFramebuffers(1, &fb_handle);
    glBindFramebuffer(GL_FRAMEBUFFER, fb_handle);

    // create texture
    glGenTextures(1, &tex_handle);
    glBindTexture(GL_TEXTURE_2D, tex_handle);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, w, h, 0, GL_RGB, GL_UNSIGNED_BYTE,
                 0);
    // set texture filtering
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

    glBindTexture(GL_TEXTURE_2D, 0);

    // attach it to currently bound framebuffer object
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
                           tex_handle, 0);

    glGenRenderbuffers(1, &rbo);
    glBindRenderbuffer(GL_RENDERBUFFER, rbo);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, w, h);
    glBindRenderbuffer(GL_RENDERBUFFER, 0);

    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT,
                              GL_RENDERBUFFER, rbo);

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
      printf("Failed to create framebuffer %d (%d x %d)\n", fb_handle, w, h);

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
  }

  void use() {
    glBindFramebuffer(GL_FRAMEBUFFER, fb_handle);
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT |
            GL_DEPTH_BUFFER_BIT); // we're not using the stencil buffer now
    glEnable(GL_DEPTH_TEST);
  }

  void draw() {
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    // glViewport(0, 0, dimensions.x, dimensions.y);
  }
};

struct TextureQuad {
  Texture2D texture;
  unsigned VBO;
  unsigned VAO;
  unsigned EBO;
  std::vector<float> verts;
  std::vector<unsigned> indices;

  TextureQuad() {}

  TextureQuad(const char *filename) {
    // verts = quad_textured_verts;
    // indices = quad_textured_indices;
    for (unsigned i = 0;
         i < sizeof(quad_textured_verts) / sizeof(quad_textured_verts[0]);
         i++) {
      verts.push_back(quad_textured_verts[i]);
    }
    for (unsigned i = 0;
         i < sizeof(quad_textured_indices) / sizeof(quad_textured_indices[0]);
         i++) {
      indices.push_back((unsigned)quad_textured_verts[i]);
    }
    bind_buffers(quad_textured_verts, sizeof(quad_textured_verts),
                 quad_textured_indices, sizeof(quad_textured_indices));
    texture = Texture2D(filename);
  }

  TextureQuad(const Texture2D &tex, std::vector<float> &verts,
              std::vector<unsigned> &indices) {
    this->verts = verts;
    this->indices = indices;
    bind_buffers(&verts.front(), verts.size(), &indices.front(),
                 indices.size());

    texture = tex;
  }

  void draw() {
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    texture.activate();
    glBindVertexArray(VAO);

    glDrawElements(GL_TRIANGLES, (GLsizei)indices.size(), GL_UNSIGNED_INT, 0);
  }

  void bind_buffers(float *verts, size_t vsize, unsigned *indices,
                    size_t isize) {
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vsize, verts, GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, isize, indices, GL_STATIC_DRAW);
    // position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float),
                          (void *)0);
    glEnableVertexAttribArray(0);
    // index attribute
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float),
                          (void *)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
  }

  void bind_buffers() {
    bind_buffers(&verts.front(), verts.size() * sizeof(float), &indices.front(),
                 indices.size() * sizeof(unsigned));
  }

  static void join(const TextureQuad &a, const TextureQuad &b,
                   TextureQuad &out) {
    assert(a.texture.id == b.texture.id);

    std::vector<float> n_verts;
    std::vector<unsigned> n_indices;

    n_verts.insert(n_verts.begin(), a.verts.begin(), a.verts.end());
    n_verts.insert(n_verts.end(), b.verts.begin(), b.verts.end());
    n_indices.insert(n_indices.begin(), a.indices.begin(), a.indices.end());
    n_indices.insert(n_indices.end(), b.indices.begin(), b.indices.end());

    auto it = max_element(std::begin(a.indices), std::end(a.indices));
    for (auto i = a.indices.size(); i < n_indices.size(); i++) {
      n_indices[i] += *it;
    }

    out = TextureQuad(a.texture, n_verts, n_indices);
  }

  static void subdivide(const TextureQuad &in, TextureQuad &out, const int n) {
    out.verts.clear();
    out.indices.clear();

    std::vector<float> verts_top_row;
    for (int i = 0; i <= n; i++) {
      float t = (float)i / n;
      verts_top_row.push_back(0.5f);
      verts_top_row.push_back(0);
      verts_top_row.push_back(0.5f - t);
      verts_top_row.push_back(1.0f);
      verts_top_row.push_back(t);
    }

    // add n verts to each row
    for (int i = 0; i <= n; i++) {
      out.verts.insert(out.verts.end(), verts_top_row.begin(),
                       verts_top_row.end());
      float t = (float)i / n;
      for (unsigned j = 0; j < verts_top_row.size(); j += 5) {
        out.verts[i * verts_top_row.size() + j] -= t;
        out.verts[i * verts_top_row.size() + j + 3] -= t;
      }
    }

    // add triangle indices
    for (int i = 0; i < n; i++)
      for (int j = 0; j < n; j++) {
        out.indices.push_back(i * (n + 1) + j);
        out.indices.push_back(i * (n + 1) + j + 1);
        out.indices.push_back((i + 1) * (n + 1) + j + 1);

        out.indices.push_back(i * (n + 1) + j);
        out.indices.push_back((i + 1) * (n + 1) + j + 1);
        out.indices.push_back((i + 1) * (n + 1) + j);
      }

    out.bind_buffers(&out.verts.front(), out.verts.size() * sizeof(float),
                     &out.indices.front(),
                     out.indices.size() * sizeof(unsigned));
    out.texture = in.texture;
  }

  static void wrap_cylinder(const TextureQuad &in, TextureQuad &out, float r) {
    std::vector<float> v = in.verts;
    for (unsigned i = 0; i < v.size(); i += 5) {
      v[i + 1] = (float)(-r * sin((v[i] + 0.5) * M_PI));
    }
    out.verts = v;
    out.bind_buffers();
  }

  static void wrap_dome(const TextureQuad &in, TextureQuad &out, float r) {
    // x^2 + y^2 + z^2 = r^2
    // x, y, r are known => z = sqrt(r^2-x^2-y^2)
    std::vector<float> v = in.verts;
    for (unsigned i = 0; i < v.size(); i += 5) {
      v[i + 1] =
          sqrt(clamp(r * r - v[i] * v[i] - v[i + 2] * v[i + 2], 0.0f, r * r));
      // DUMP(v[i + 1]);
    }
    out.verts = v;
    out.bind_buffers();
  }
};

enum CameraMode { CAMERA_ORBIT, CAMERA_FLY };

struct Camera {
  CameraMode type = CAMERA_ORBIT;
  float fov = 45.0f;
  float ang = (float)M_PI_2;
  float rad = 3;
  float height = 6;
  glm::vec3 pos = glm::vec3(cosf(ang) * rad, height, sinf(ang) * rad);
  glm::vec3 front = glm::vec3(1.0f, 0.0f, 0.0f);
  glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f);
  glm::vec3 target = glm::vec3(0);

  glm::mat4 view;
  glm::mat4 projection;

  bool firstMouse = true;
  float yaw = -90.0f; // yaw is initialized to -90.0 degrees since a yaw of 0.0
                      // results in a direction vector pointing to the right so
                      // we initially rotate a bit to the left.
  float pitch = 0.0f;

  void get_pixelray(const glm::vec2 &pixel, const glm::vec2 &win_size_pixels,
                    glm::vec3 &ray_start, glm::vec3 &ray_end) const {
    glm::vec2 pix_normalized(pixel.x / win_size_pixels.x,
                             1.0f - pixel.y / win_size_pixels.y);
    ray_start = glm::unProject(glm::vec3(pix_normalized, 0.0f), view,
                               projection, glm::vec4(0, 0, 1, 1));
    ray_end = glm::unProject(glm::vec3(pix_normalized, 1.0f), view, projection,
                             glm::vec4(0, 0, 1, 1));
  }
};

void draw_arrays(const glm::vec3 &position, const glm::vec3 &scale,
                 const Shader &shader, GLsizei vert_count,
                 int mode = GL_TRIANGLES,
                 const glm::vec3 &ax = glm::vec3(0, 1, 0), float ang = 0) {
  glm::mat4 model = glm::mat4(1.0f);
  model = glm::translate(model, position);
  model = glm::scale(model, scale);
  // float angle = 20.0f * i;
  model = glm::rotate(model, ang, ax);
  shader.setMat4("model", model);
  glDrawArrays(mode, 0, vert_count);
}

void draw_arrays(const glm::mat3 &model, const Shader &shader,
                 GLsizei vert_count, int mode = GL_TRIANGLES) {
  shader.setMat4("model", model);
  glDrawArrays(mode, 0, vert_count);
}

void draw_OBB(const OBB &box, const Shader &cam_shader) {
  glm::mat4 r(1.0f);
  glm::mat4 model(1.0f);
  glm::vec3 mscale(1, 1, 1);

  // top-left 3x3 of the rotation matrix = normalized box extents
  // scale vector = length of each box extent
  for (int i = 0; i < 3; i++) {
    mscale[i] = glm::length(box.ext[i]);
    r[i] = glm::vec4(box.ext[i] / mscale[i], 0);
  }

  model = glm::translate(model, box.pos);
  model = model * r;
  model = glm::scale(model, mscale);
  cam_shader.setMat4("model", model);

  glDrawArrays(GL_TRIANGLES, 0, 36);
}

void draw_grid(unsigned int gridVAO, Shader &cam_shader, float width = 1,
               int rows = 10, int cols = 10,
               const glm::vec3 &offset = glm::vec3(0)) {
  glBindVertexArray(gridVAO);
  cam_shader.setVec4("color", glm::vec4(1.0f, 1.0f, 1.0f, 0.6f));
  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  glm::vec3 gridscale(width);
  for (int r = -rows; r < rows + 1; r++)
    for (int c = -cols; c < cols + 1; c++) {
      glm::vec3 gridpos(r * width, 0, c * width);
      gridpos += offset;
      gl::draw_arrays(gridpos, gridscale, cam_shader, 4, GL_LINE_STRIP);
    }
}

void draw_axes(unsigned int axVAO, Shader &cam_shader) {
  glBindVertexArray(axVAO);
  glm::vec3 pos(0, 0.01, 0);
  glm::vec3 scale(1);
  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

  for (int i = 0; i < 3; i++) {
    glm::vec3 color(0);
    color[i] = 1;
    cam_shader.setVec4("color", glm::vec4(color, 1.0f));
    glm::vec3 axis(1, 0, 0);
    if (i != 0) {
      axis = glm::vec3(0);
      axis[i - 1] = 1;
    }

    draw_arrays(pos, scale, cam_shader, 2, GL_LINES, axis, (float)M_PI_2);
  }
}

void draw_path(unsigned graphVAO, Shader &cam_shader, std_vec2d<bool> &graph,
               glm::vec3 &offset, float side_len = 1,
               const std::vector<GridNode> path = std::vector<GridNode>()) {
  glm::vec3 pos;
  glm::vec3 def_scale(side_len);
  glm::vec3 origin(0);
  glm::vec4 obstaclecolor(0.5f, 0.5f, 0.5f, 1.0f);
  glm::vec4 pathcolor(0.2f, 0.8f, 0.2f, 1.0f);
  glm::vec4 startcolor(0.2f, 0.2f, 0.8f, 1.0f);
  glm::vec4 goalcolor(0.8f, 0.2f, 0.2f, 1.0f);

  cam_shader.use();

  auto draw_cmd = [&](int x, int y, const glm::vec4 &color) {
    if (XN_LIT) {
      cam_shader.setVec3("objectColor", glm::vec3(color.x, color.y, color.z));
    } else {
      cam_shader.setVec4("color", color);
    }
    pos = {x * side_len, 0, y * side_len};
    pos += offset;
    pos.y = -side_len * 0.5f - 0.01f;
    gl::draw_arrays(pos, def_scale, cam_shader, 36);
  };

  glBindVertexArray(graphVAO);
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

  if (path.size() > 1) {
    for (auto i = path.begin() + 1; i < path.end() - 1; i++)
      draw_cmd(i->x, i->y, pathcolor);
    draw_cmd(path.front().x, path.front().y, startcolor);
    draw_cmd(path.back().x, path.back().y, goalcolor);
  }
}

void gen_arrays(unsigned int *VAO, unsigned int *VBO, float *vertex_array,
                size_t vertex_array_length, int stride) {
  glGenVertexArrays(1, VAO);
  glGenBuffers(1, VBO);
  glBindVertexArray(*VAO);
  glBindBuffer(GL_ARRAY_BUFFER, *VBO);
  glBufferData(GL_ARRAY_BUFFER, vertex_array_length, vertex_array,
               GL_STATIC_DRAW);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride * sizeof(float),
                        (void *)0);
  glEnableVertexAttribArray(0);
}

void gen_arrays(float *vertex_array, size_t vertex_array_length,
                VertexArrayInfo &out, int stride = 3) {
  out.array = vertex_array;
  out.length = vertex_array_length;
  out.stride = stride;

  gen_arrays(&out.VAO, &out.VBO, out.array, out.length, out.stride);
}

void gen_rayscan_verts(const std::vector<glm::vec3> &in_vec,
                       gl::VertexArrayInfo &out_info) {
  if (in_vec.size() == 0)
    return;

  std::vector<glm::vec3> sorted;
  sort_rays_by_angle(in_vec, sorted);

  float *verts = new float[sorted.size() * 3];

  for (unsigned int i = 0; i < sorted.size(); i++) {
    for (int j = 0; j < 3; j++)
      verts[i * 3 + j] = sorted[i][j];
  }

  gl::gen_arrays(verts, sorted.size() * sizeof(float) * 3, out_info);
}

void gen_dome_verts(const int num_rings, const int verts_per_ring,
                    const float r, std::vector<glm::vec3> &out) {
  std::vector<glm::vec3> ring_verts;
  out.clear();

  for (int i = 0; i < num_rings; i++) {
    for (int j = 0; j < verts_per_ring; j++) {
      float ang = 2 * (float)M_PI * j / verts_per_ring;
      ring_verts.push_back(glm::vec3(cos(ang), (float)i / num_rings, sin(ang)));
      ring_verts.back() *= (float)i * r / num_rings;
      ring_verts.back().y = r - ring_verts.back().y;
    }
  }

  out = ring_verts;
}

void begin_3d(const Camera &cam, Shader &s) {
  s.use();
  s.setMat4("view", cam.view);
  s.setMat4("projection", cam.projection);
}

void cls(float r = 0, float g = 0, float b = 0, float a = 1.0f) {
  glClearColor(r, g, b, a);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

} // namespace gl
} // namespace xn