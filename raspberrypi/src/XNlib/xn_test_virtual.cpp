#define PIO_VIRTUAL

#include "assert.h"
#include "opengl/xn_gl.hpp"
#include "xn_ik.hpp"
#include <cmath>

#include <fenv.h>

// g++ ./opengl/glad/src/glad.c xn_test_virtual.cpp -I./opengl/glad/include
// -lglfw3 -lGL -lX11 -lpthread -lXrandr -lXi -ldl -lXxf86vm -o gl_app -g

// arm pos : (245, -170) mm,
// target : (400, -25) mm

using namespace xn;
using namespace std;

void processInput(GLFWwindow *window);

#ifdef PIO_VIRTUAL
void draw_box(Shader &shader, vec3 &position, glm::vec4 color,
              vec3 scale = {1, 1, 1}, bool isWireframe = false) {
  glm::vec3 gpos = position.toGLM();
  glm::vec3 gscale = scale.toGLM();
  shader.setVec4("color", color);
  if (isWireframe)
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  else
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  gl::draw_arrays(gpos, gscale, shader, 36);
}
#endif

// settings
const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;
int mode = 0;
int max_mode = 4;

// camera
float camera_ang = 0;
float camera_rad = 3;
float camera_height = 2;
glm::vec3 cameraPos = glm::vec3(cosf(camera_ang) * camera_rad, camera_height,
                                sinf(camera_ang) * camera_rad);
glm::vec3 cameraFront = glm::vec3(1.0f, 0.0f, 0.0f);
glm::vec3 cameraUp = glm::vec3(0.0f, 1.0f, 0.0f);

float yaw = -90.0f;
float pitch = 0.0f;

// timing
float deltaTime = 0.0f; // time between current frame and last frame
float lastFrame = 0.0f;

float arm_len = 104; // in milimeters

void stop(int signum) { run = 0; }

int main() {
  // feenableexcept(FE_INVALID | FE_OVERFLOW);

#ifndef PIO_VIRTUAL
  if (gpioInitialise() < 0)
    return -1;
  gpioSetSignalFunc(SIGINT, stop);
#endif
  // vec3 target{0,1.4, 0};

  vec3 pole = {0, 0, 1};
  Transform bonechain[] = {Transform({0, 0, 0}), Transform({0, 1, 0}),
                           Transform({0, 1 + 7.5 / 18, 0}),
                           Transform({0, 1 + 15.0 / 18, 0})};
  float start_bone_lengths[] = {
      vec3::dist(bonechain[0].position, bonechain[1].position),
      vec3::dist(bonechain[1].position, bonechain[2].position)};

  ik::IkChain arm(4, bonechain, pole);

  // phys ik test code
  // vector<pio::ServoAngular> servos[] = {
  //   vector<pio::ServoAngular>(),
  //   vector<pio::ServoAngular>(),
  //   vector<pio::ServoAngular>(),
  //   vector<pio::ServoAngular>()
  // };
  // servos[0].push_back(pio::ServoAngular(12, 500, 2500));
  // servos[0].push_back(pio::ServoAngular(13, 500, 2500));
  // servos[1].push_back(pio::ServoAngular(26, 575, 2460));
  // servos[2].push_back(pio::ServoAngular(16, 575, 2460));

  // printf("%f %f\n", servos[0][1].min_angle, servos[0][1].max_angle);

  // vector<vec3> axes[] = {
  //   vector<vec3>(),
  //   vector<vec3>(),
  //   vector<vec3>(),
  //   vector<vec3>()
  // };

  // axes[0].push_back({0,1,0});
  // axes[0].push_back({1,0,0});
  // axes[1].push_back({1,0,0});
  // axes[2].push_back({-1,0,0});
  std::vector<xn::pio::SmoothServo> servos[] = {
      std::vector<xn::pio::SmoothServo>(), std::vector<xn::pio::SmoothServo>(),
      std::vector<xn::pio::SmoothServo>(), std::vector<xn::pio::SmoothServo>()};

  servos[0].push_back(xn::pio::SmoothServo(xn::pio::ServoAngular(12, 500, 2500),
                                           {0, -1, 0}, bonechain[0].position,
                                           1500));
  servos[0].push_back(xn::pio::SmoothServo(xn::pio::ServoAngular(13, 500, 2500),
                                           {1, 0, 0}, bonechain[0].position,
                                           1570));
  servos[1].push_back(xn::pio::SmoothServo(xn::pio::ServoAngular(26, 500, 2500),
                                           {1, 0, 0}, bonechain[1].position,
                                           1420));
  servos[2].push_back(xn::pio::SmoothServo(xn::pio::ServoAngular(16, 500, 2500),
                                           {-1, 0, 0}, bonechain[2].position,
                                           1500));

  ik::ServoChain phys_arm(arm, servos, &run, arm_len);
  phys_arm.reset();
  // time_sleep(0.5);
  start_bone_lengths[0] =
      vec3::dist(phys_arm.positions[0], phys_arm.positions[1]);
  start_bone_lengths[1] =
      vec3::dist(phys_arm.positions[1], phys_arm.positions[2]);

  // init opengl
  GLFWwindow *window = xn::gl::CreateWindow(SCR_WIDTH, SCR_HEIGHT, "boxin");
  glEnable(GL_DEPTH_TEST);
  Shader cam_shader("opengl/shaders/camera.vs", "opengl/shaders/camera.fs");
  unsigned int VBO, VAO, VBO_grid, VAO_grid, VBO_curve, VAO_curve;
  gl::gen_arrays(&VAO, &VBO, gl::cube_verts, sizeof(gl::cube_verts), 5);
  gl::gen_arrays(&VAO_grid, &VBO_grid, gl::grid_verts, sizeof(gl::grid_verts),
                 3);

  float curve_verts[100 * 3];

  // generate bezier points
  BezierCurve b;
  for (int i = 0; i < 100; i++) {
    vec2 res = b.solve(i / 100.0);
    curve_verts[i * 3] = res.x;
    curve_verts[i * 3 + 1] = res.y;
    curve_verts[i * 3 + 2] = 0;
  }
  gl::gen_arrays(&VAO_curve, &VBO_curve, curve_verts, sizeof(curve_verts), 3);

  cam_shader.use();

  float tot_err = 0;
  float avg_err = 0;
  int iters = 0;
  while (run && !glfwWindowShouldClose(window)) {
    // for ( int j = 0; j < 100; j++) {
    static float i = 0;
    static float iter = 0.01;

    if (i + iter > 1 || i + iter < 0)
      iter *= -1;

    i += iter;

    static float ang = 0;
    ang = ang + 0.01;

    vec3 target;
    vec3 pole;

    switch (mode) {
    case 0:
      target = {-1 + i * 2, 1, 1};
      pole = {-2 + i * 4, 1, 2};
      break;
    case 1:
      target = {0, 1.3 + i, 0};
      pole = {0, 0, 1};
      break;
    case 2:
      target = {cosf(ang) * 2, 0.2, sinf(ang) * 2};
      pole = {0, 0, 1};
      break;
    case 3:

      // vec3 arm_pos = {245, 0, -170};
      // target = {340, 0, -30};
      // target = target - arm_pos;
      // target = target / arm_len;

      target = {-2.127778, 0.000000, -0.477778};
      // target = target / arm_len;
      // target = {cos(ang)*i*3, 0, sin(ang)*i*3};
      // cout << "target: " << target;
      break;
    }

    // if ( mode < 3 )
    target = target * arm_len;

    phys_arm.resolve(target);
    // cout << "pole: " << phys_arm.ik_chain.pole << '\n';
    // cout << "bone positions\n";
    // for ( int i = 0; i < phys_arm.ik_chain.bone_count; i++ ) {
    //   cout << phys_arm.positions[i] << '\n';
    // }

    tot_err +=
        vec3::dist(phys_arm.positions[2], phys_arm.ik_chain.positions[2]);
    avg_err = tot_err / (++iters);

    // render
    float currentFrame = glfwGetTime();
    deltaTime = currentFrame - lastFrame;
    lastFrame = currentFrame;
    processInput(window);
    glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glm::mat4 projection =
        glm::perspective(glm::radians(xn::gl::fov),
                         (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);
    cam_shader.setMat4("projection", projection);
    glm::mat4 view = glm::lookAt(cameraPos, glm::vec3(0, 1, 0), cameraUp);
    cam_shader.setMat4("view", view);
    glBindVertexArray(VAO);

    vec3 joint_scale{0.1, 0.1, 0.1};
    for (int j = 0; j < arm.bone_count; j++) {
      draw_box(cam_shader, phys_arm.ik_chain.positions[j],
               glm::vec4(1.0f, 0.5f, 0.2f, 1.0f), joint_scale, true);
    }
    for (int j = 0; j < arm.bone_count; j++) {
      draw_box(cam_shader, phys_arm.positions[j],
               glm::vec4(1.0f, 0.0f, 0.0f, 1.0f), joint_scale);
    }

    vec3 pole_scale{0.05, 0.05, 0.05};
    vec3 ntarget = target / arm_len;
    for (int j = 0; j < arm.bone_count - 2; j++)
      draw_box(cam_shader, phys_arm.ik_chain.poles[j],
               glm::vec4(0.0f, 1.0f, 0.0f, 1.0f), pole_scale);
    draw_box(cam_shader, ntarget, glm::vec4(0.0f, 0.0f, 1.0f, 1.0f),
             pole_scale);

    gl::draw_grid(VAO_grid, cam_shader);

    // draw bezier curve
    // glBindVertexArray(VAO_curve);
    // cam_shader.setVec4("color", glm::vec4(1.0f, 1.0f, 0.0f, 1.0f));
    // glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    // glm::vec3 gridscale(1, 1, 1);
    // glm::vec3 gridpos(0,0,0);
    // gl::draw_arrays(gridpos, gridscale, cam_shader, 100, GL_LINE_STRIP);
    // }

    glfwSwapBuffers(window);
    glfwPollEvents();

    time_sleep(1.0 / 60);
  }

  glDeleteVertexArrays(1, &VAO);
  glDeleteBuffers(1, &VBO);
  glfwTerminate();

  printf("\n");
  DUMP(avg_err);

  std::string dummy;
  std::cout << "Enter to continue..." << std::endl;
  std::getline(std::cin, dummy);
}

void processInput(GLFWwindow *window) {
  if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
    glfwSetWindowShouldClose(window, true);
    run = 0;
  }

  float cameraSpeed = 2.5 * deltaTime;
  if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) {
    camera_rad -= cameraSpeed;
  }
  if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) {
    camera_rad += cameraSpeed;
  }
  if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) {
    camera_ang += cameraSpeed;
  }
  // cameraPos -= glm::normalize(glm::cross(cameraFront, cameraUp)) *
  // cameraSpeed;
  if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) {
    camera_ang -= cameraSpeed;
  }

  static bool edown = false;
  if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS && !edown) {
    mode = (mode + 1) % max_mode;
    edown = true;
  } else if (glfwGetKey(window, GLFW_KEY_E) == GLFW_RELEASE) {
    edown = false;
  }
  cameraPos = glm::vec3(cosf(camera_ang) * camera_rad, camera_height,
                        sinf(camera_ang) * camera_rad);
}