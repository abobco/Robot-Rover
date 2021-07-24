/*
    High level renderer, ideally independent of graphics backend

    problems:
    - Possible vertex array formats:
        - XYZ
        - XYZ UV
        - XYZ RGB
        - XYZ RGBA
        - XYZ RGB UV
        - XYZ RGBA UV

    - optional index array

    - polygon modes:
        - front and back
        - fill
        - lines

    OpenGL review:

        Steps to draw anything in opengl:
        Setup:
            1. Create window
            2. assign opengl context to window
            3. Generate vertex arrays
            4. Bind vertex data to vertex buffers. We now have 2 ints(VAO, VBO)
   to reference the data
            5. Set vertex attribute pointer. This is where the vertex array
   format must be given and saved 5.b (optional)  repeat steps 3-5 w/ indices
   array, get EBO handle
            6. Compile shaders
        Loop:
            1. In any order:
                - Set shader uniforms
                - Bind vertex array
                - Activate shader
            2. call glDrawArrays(mode, first, count) or glDrawElements(mode,
   count, type, indices)

    Default shader formats:
        3D:
        - uniforms: mat4 model, view, projection
        - set all 3 before every call to glDrawArrays

        2D:
        - uniforms: mat4 projection,
                    vec2 position, scale
                    vec3 color
                    float angle
        - position is in pixel coordinates, projection is set accordingly

        Text:
        - uniforms: mat4 projection
                    vec2 position, scale
                    vec3 color
                    sampler2D text
        - position is in pixel coordinates, projection is set accordingly
*/
#pragma once
#include "xn_gl.hpp"
#include <type_traits>
#include <unordered_map>
#include <utility>

namespace xn {
enum VertexFormat {
  VERTEX_XYZ,
  VERTEX_XYZ_UV,
  VERTEX_XYZ_RGB,
  VERTEX_XYZ_RGBA,
  VERTEX_XYZ_RGB_UV,
  VERTEX_XYZ_NORMAL
};

struct Mesh {
  VertexFormat vert_format;
  std::vector<float> verts;
  std::vector<unsigned> indices;
  unsigned VAO, VBO, EBO;
  unsigned stride = 1;
  GLenum draw_mode = GL_TRIANGLES;
  std::pair<GLenum, GLenum> polygon_mode =
      std::pair(GL_FRONT_AND_BACK, GL_FILL);

  Mesh() {}

  Mesh(const std::vector<glm::vec3> &verts, VertexFormat format = VERTEX_XYZ,
       const std::vector<unsigned> &indices = std::vector<unsigned>()) {
    std::vector<float> v;
    if (verts.size() == 0)
      return;
    v.insert(v.begin(), (float *)&verts.front(),
             (float *)&verts.front() + verts.size() * 3);
    init(v, format, indices);
  }

  Mesh(const std::vector<float> &verts, VertexFormat format = VERTEX_XYZ,
       const std::vector<unsigned> &indices = std::vector<unsigned>()) {
    init(verts, format, indices);
  }

  virtual ~Mesh() {
    verts.clear();
    indices.clear();
  }

  virtual void
  init(const std::vector<float> &verts, VertexFormat format = VERTEX_XYZ,
       const std::vector<unsigned> &indices = std::vector<unsigned>()) {
    this->vert_format = format;
    this->verts = verts;
    this->indices = indices;

    if (verts.size() == 0)
      return;

    glPolygonMode(polygon_mode.first, polygon_mode.second);
    switch (format) {
    case VERTEX_XYZ:
      glGenVertexArrays(1, &VAO);
      glGenBuffers(1, &VBO);
      glBindVertexArray(VAO);
      glBindBuffer(GL_ARRAY_BUFFER, VBO);
      glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(float),
                   &verts.front(), GL_STATIC_DRAW);
      glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float),
                            (void *)0);
      glEnableVertexAttribArray(0);
      stride = 3;
      break;
    case VERTEX_XYZ_UV:
      glGenVertexArrays(1, &VAO);
      glGenBuffers(1, &VBO);
      glGenBuffers(1, &EBO);

      glBindVertexArray(VAO);

      if (verts.size() > 0) {
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(float),
                     &verts.front(), GL_STATIC_DRAW);
      }
      if (indices.size() > 0) {
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned),
                     &indices.front(), GL_STATIC_DRAW);
      }

      // position attribute
      glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float),
                            (void *)0);
      glEnableVertexAttribArray(0);

      if (indices.size() > 0) {
        // index attribute
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float),
                              (void *)(3 * sizeof(float)));
        glEnableVertexAttribArray(1);
      }
      stride = 5;
      break;
    case VERTEX_XYZ_NORMAL:
    case VERTEX_XYZ_RGB:
      glGenVertexArrays(1, &VAO);
      glGenBuffers(1, &VBO);
      glBindVertexArray(VAO);
      glBindBuffer(GL_ARRAY_BUFFER, VBO);
      glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(float),
                   &verts.front(), GL_STATIC_DRAW);
      // glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float),
      // (void *)0); glEnableVertexAttribArray(0);

      // position attribute
      glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                            (void *)0);
      glEnableVertexAttribArray(0);
      // color attribute
      glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                            (void *)(3 * sizeof(float)));
      glEnableVertexAttribArray(1);
      stride = 6;
      break;
    default:
      break;
    }
  }

  void setPolygonMode(GLenum face, GLenum mode) {
    polygon_mode = std::pair(face, mode);
  }

  void setDrawMode(GLenum mode) { draw_mode = mode; }

  virtual void draw() {
    glPolygonMode(polygon_mode.first, polygon_mode.second);
    glBindVertexArray(VAO);
    glDrawArrays(draw_mode, 0, (GLsizei)verts.size() / stride);
  }

  static Mesh *gen_grid(float side_len = 1, int rows = 10, int cols = 10) {
    std::vector<float> verts;
    verts.reserve(3 * 2 * (rows + cols));
    float halflen_rows = 0.5f * rows * side_len;
    float halflen_cols = 0.5f * cols * side_len;
    for (int i = 0; i < rows; i++) {
      verts.push_back(-halflen_rows + i * side_len);
      verts.push_back(0);
      verts.push_back(-halflen_cols);

      verts.push_back(-halflen_rows + i * side_len);
      verts.push_back(0);
      verts.push_back(halflen_cols);
    }

    for (int i = 0; i < cols; i++) {
      verts.push_back(-halflen_rows);
      verts.push_back(0);
      verts.push_back(-halflen_cols + i * side_len);

      verts.push_back(halflen_rows);
      verts.push_back(0);
      verts.push_back(-halflen_cols + i * side_len);
    }

    Mesh *m = new Mesh(verts);
    m->draw_mode = GL_LINES;
    m->polygon_mode.second = GL_LINE;

    return m;
  }

  static Mesh *gen_box_batch(std::vector<glm::vec3> positions,
                             glm::vec3 scale = glm::vec3(0.05, 0.05, 0.05)) {
    const size_t cubesize = sizeof(gl::cube_verts_lit) / sizeof(float);
    std::vector<float> verts;
    verts.reserve(cubesize * positions.size());

    for (const glm::vec3 &p : positions) {
      float new_verts[cubesize];
      std::copy(std::begin(gl::cube_verts_lit), std::end(gl::cube_verts_lit),
                std::begin(new_verts));
      for (unsigned i = 0; i < cubesize; i += 6) {
        new_verts[i] *= scale.x;
        new_verts[i + 1] *= scale.y;
        new_verts[i + 2] *= scale.z;

        new_verts[i] += p.x;
        new_verts[i + 1] += p.y;
        new_verts[i + 2] += p.z;
      }

      verts.insert(verts.begin(), std::begin(new_verts), std::end(new_verts));
    }

    Mesh *m = new Mesh(verts, xn::VERTEX_XYZ_NORMAL);
    // m->setPolygonMode(GL_FRONT, GL_FILL)
    return m;
  }
};

struct BoxMesh : public Mesh {
  BoxMesh() { stride = 6; }

  void init() {
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(gl::cube_verts_lit),
                 gl::cube_verts_lit, GL_STATIC_DRAW);
    // position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                          (void *)0);
    glEnableVertexAttribArray(0);
    // normal attribute
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                          (void *)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
  }

  void draw() {
    // printf("drawfail\n");
    glPolygonMode(polygon_mode.first, polygon_mode.second);
    glBindVertexArray(VAO);
    glDrawArrays(draw_mode, 0, 36);
  }
};

struct UniformBase {
  std::string name = "";
  UniformBase() {}
  UniformBase(std::string name) { this->name = name; }
  virtual void set(const Shader &shader) const {}
  virtual void assign_value(){};
  virtual ~UniformBase(){};
};

template <typename T> struct Uniform : public UniformBase {
  T value;

  Uniform() {}

  Uniform(std::string name, T value) {
    this->name = name;
    this->value = value;
  }

  void assign_value(T value) { this->value = value; };

  void set(const Shader &shader) const {
    if constexpr (std::is_same_v<T, bool>)
      shader.setBool(name, value);
    else if constexpr (std::is_same_v<T, int>)
      shader.setInt(name, value);
    else if constexpr (std::is_same_v<T, float>)
      shader.setFloat(name, value);
    else if constexpr (std::is_same_v<T, glm::vec2>)
      shader.setVec2(name, value);
    else if constexpr (std::is_same_v<T, glm::vec3>)
      shader.setVec3(name, value);
    else if constexpr (std::is_same_v<T, glm::vec4>)
      shader.setVec4(name, value);
    else if constexpr (std::is_same_v<T, glm::mat2>)
      shader.setMat2(name, value);
    else if constexpr (std::is_same_v<T, glm::mat3>)
      shader.setMat3(name, value);
    else if constexpr (std::is_same_v<T, glm::mat4>)
      shader.setMat4(name, value);
  }

  ~Uniform() {}
};

struct DrawNode {
  static DrawNode scene;
  static int id_count;
  static std::unordered_map<std::string, Shader> shader_map;
  static std::unordered_map<std::string, DrawNode *> node_map;

  int id;
  DrawNode *parent = NULL;
  std::vector<DrawNode *> children = std::vector<DrawNode *>();

  glm::mat4 model = glm::mat4(1);
  glm::vec3 position = glm::vec3(0);

  bool visible = true;
  Shader *shader = NULL;
  std::vector<UniformBase *> uniforms;
  std::vector<Mesh *> meshes;

  DrawNode() { id = id_count++; }

  DrawNode(Shader *shader) {
    id = id_count++;
    this->shader = shader;
  }

  void addMesh(Mesh *mesh) { meshes.push_back(mesh); }

  DrawNode(Shader *shader, Mesh *mesh) {
    id = id_count++;
    this->shader = shader;
    addMesh(mesh);
  }

  DrawNode *addChild(DrawNode *c) {
    // c.parent = this;
    children.push_back(c);
    return children.back();
  }

  DrawNode *popChild(int id) {
    for (unsigned i = 0; i < children.size(); i++) {
      DrawNode *c = children[i];
      if (id == c->id) {
        children.erase(children.begin() + i);
        children.resize(children.size() - 1);
        return c;
      }
    }

    for (unsigned i = 0; i < children.size(); i++) {
      DrawNode *c = children[i]->popChild(id);
      if (c != NULL)
        return c;
    }

    return NULL;
  }

  void setModelRecursive(glm::mat4 m = glm::mat4(1)) {
    // DUMP(children.size());
    model = m;
    for (DrawNode *c : children) {
      c->setModelRecursive(model);
    }
  }

  void cleanupRecursive() {
    for (UniformBase *u : uniforms)
      delete u;

    for (Mesh *m : meshes)
      delete m;

    for (DrawNode *c : children) {
      c->cleanupRecursive();
    }

    if (parent != NULL) {
      parent->popChild(id);
      delete this;
    }
  }

  int countChildrenRecursive() {
    int child_count = 0;
    for (DrawNode *c : children) {
      child_count++;
      child_count += c->countChildrenRecursive();
    }
    return child_count;
  }

  void transform(glm::vec3 translation, glm::vec3 scale = glm::vec3(1),
                 float angle = 0.0f, glm::vec3 axis = glm::vec3(0, 1, 0)) {
    position += translation;
    model = glm::translate(model, translation);
    model = glm::scale(model, scale);

    if (parent != NULL) {
      model = glm::translate(model, -parent->position);
      model = glm::rotate(model, angle, axis);
      model = glm::translate(model, parent->position);
    } else {
      model = glm::rotate(model, angle, axis);
    }

    for (DrawNode *c : children) {
      c->transform(translation, scale, angle, axis);
    }
  }

  template <typename T> void addUniform(std::string name, T value) {
    uniforms.push_back(new Uniform<T>(name, value));
  }

  template <typename T> void setUniformByName(std::string name, T value) {
    for (UniformBase *u : uniforms) {
      if (u->name.compare(name) == 0) {
        u->assign_value(value);
        break;
      }
    }
  }

  void draw() {
    if (!visible)
      return;
    if (shader != NULL)
      shader->use();
    for (UniformBase *u : uniforms) {
      u->set(*shader);
    }

    for (Mesh *m : meshes) {
      shader->setMat4("model", model);
      m->draw();
    }

    for (DrawNode *c : children) {
      c->draw();
    }
  }
};
} // namespace xn