// #define PIO_VIRTUAL

#ifdef PIO_VIRTUAL
#include "opengl/xn_gl.hpp"
#endif
#include "assert.h"
#include "xn_ik.hpp"
#include <cmath>


#include <fenv.h>

// g++ ./glad/src/glad.c xn_test_virtual.cpp -I./glad/include -lglfw3 -lGL -lX11
// -lpthread -lXrandr -lXi -ldl -lXxf86vm -o gl_app

using namespace xn;
using namespace std;

#ifdef PIO_VIRTUAL
void processInput(GLFWwindow *window);
void time_sleep(double seconds);
void draw_bone(vec3 &position, const Shader &shader,
               ik::IkConstraint &constraint, vector<pio::ServoAngular> &servos,
               int isWireFrame = false);
void set_bone_positions(ik::ServoChain arm);
// settings
const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;
int max_mode = 3;

// camera
float camera_ang = 0;
float camera_rad = 3;
float camera_height = 2;
glm::vec3 cameraPos = glm::vec3(cosf(camera_ang) * camera_rad, camera_height,
                                sinf(camera_ang) * camera_rad);
glm::vec3 cameraFront = glm::vec3(1.0f, 0.0f, 0.0f);
glm::vec3 cameraUp = glm::vec3(0.0f, 1.0f, 0.0f);

bool firstMouse = true;
float yaw = -90.0f; // yaw is initialized to -90.0 degrees since a yaw of 0.0
                    // results in a direction vector pointing to the right so we
                    // initially rotate a bit to the left.
float pitch = 0.0f;

// timing
float deltaTime = 0.0f; // time between current frame and last frame
float lastFrame = 0.0f;

// joint translations
glm::vec3 ideal_joint_positions[3];
glm::vec3 phys_joint_positions[3];
glm::vec3 joint_scale(0.1, 0.1, 0.1);
#endif

int mode = 0;
ik::ServoChain *armptr;
bool testrun = 1;

void stop(int signum) {
  testrun = 0;
  printf("\nmoving servos back to neutral\n");
  for (int i = 0; i < 3; i++)
    for (pio::SmoothServo &s : armptr->servos[i])
      s.target_angle = M_PI / 2;
  // pthread_mutex_unlock(&sem);

  time_sleep(armptr->servos[0][0].t_max / 2);
  run = 0;
}

int main() {
  // feenableexcept(FE_INVALID | FE_OVERFLOW);

#ifndef PIO_VIRTUAL
  if (gpioInitialise() < 0)
    return -1;
  gpioSetSignalFunc(SIGINT, stop);
#endif

  // ik::IkChain arm(3, bonechain, pole);
  vec3 pole = {0, 0, 1};
  vec3 bonechain[] = {vec3{0, 0, 0}, vec3{0, 1, 0}, vec3{0, 1 + 7.5 / 18, 0},
                      vec3{0, 1 + 15.0 / 18, 0}};
  float start_bone_lengths[] = {vec3::dist(bonechain[0], bonechain[1]),
                                vec3::dist(bonechain[1], bonechain[2])};

  ik::IkChain arm(4, bonechain, pole);

  std::vector<xn::pio::SmoothServo> servos[] = {
      std::vector<xn::pio::SmoothServo>(), std::vector<xn::pio::SmoothServo>(),
      std::vector<xn::pio::SmoothServo>(), std::vector<xn::pio::SmoothServo>()};

  servos[0].push_back(xn::pio::SmoothServo(xn::pio::ServoAngular(12, 500, 2500),
                                           {0, -1, 0}, bonechain[0], 1500));
  servos[0].push_back(xn::pio::SmoothServo(xn::pio::ServoAngular(13, 500, 2500),
                                           {1, 0, 0}, bonechain[0], 1570));
  servos[1].push_back(xn::pio::SmoothServo(xn::pio::ServoAngular(26, 500, 2500),
                                           {1, 0, 0}, bonechain[1], 1420));
  servos[2].push_back(xn::pio::SmoothServo(xn::pio::ServoAngular(16, 500, 2500),
                                           {1, 0, 0}, bonechain[2], 1500));

  ik::ServoChain phys_arm(arm, servos, &run);
  armptr = &phys_arm;
  phys_arm.reset();

  xn::pio::SmoothServo wrist(xn::pio::ServoAngular(6, 500, 2500));
  pthread_create(&wrist.tid, NULL, xn::ik::move_servo_thread, &wrist);
  // time_sleep(0.5);
  start_bone_lengths[0] =
      vec3::dist(phys_arm.positions[0], phys_arm.positions[1]);
  start_bone_lengths[1] =
      vec3::dist(phys_arm.positions[1], phys_arm.positions[2]);

// init opengl
#ifdef PIO_VIRTUAL
  GLFWwindow *window = xn::gl::CreateWindow(SCR_WIDTH, SCR_HEIGHT, "boxin");
  glEnable(GL_DEPTH_TEST);
  Shader cam_shader("opengl/shaders/camera.vs", "opengl/shaders/camera.fs");
  set_bone_positions(phys_arm);
  unsigned int VBO, VAO, VBO_grid, VAO_grid;
  gl::gen_arrays(&VAO, &VBO, gl::cube_verts, sizeof(gl::cube_verts), 5);
  gl::gen_arrays(&VAO_grid, &VBO_grid, gl::grid_verts, sizeof(gl::grid_verts),
                 3);
  cam_shader.use();
#endif

  float tot_err = 0;
  float avg_err = 0;
  int iters = 0;
  while (testrun) {
    // for ( int j = 0; j < 100; j++) {
    static float i = 0;
    static float iter = 0.01;

    if (i + iter > 1 || i + iter < 0)
      iter *= -1;

    i += iter;

    static float ang = 0;
    ang += 0.01;

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
      target = {cosf(ang) * 2, 0.5, sinf(ang) * 2};
      pole = {0, 0, 1};
      break;
    case 3:
      // vec3 arm_pos = {225, 0, 265};
      // target = {-158, 0, 179};
      // target = target - arm_pos;
      // target = {cos(ang)*i*3, 0, sin(ang)*i*3};
      // target = target / 180;
      // target = {0,0,0.5};
      // target ={-2.127778, 0.000000, -0.477778};

      target = {0, 0.5, -1};
      break;
    }
    // ang = fmod(ang+0.01, M_PI);

    // vec3 target{cosf(ang)*10, 10, sinf(ang)*10};
    // vec3 target{0, 1.3+ i, 0};
    // vec3 target{-1+i*2 , 1, 1};

    // printf("pole = ");
    // pole.print();

    printf("target = ");
    target.print();
    phys_arm.resolve(target);
    // cout << "bone positions\n";
    // for ( int i = 0; i < phys_arm.ik_chain.bone_count; i++ ) {
    //   cout << phys_arm.positions[i] << '\n';
    // }

    float ac = 0;
    for (int i = 0; i < phys_arm.ik_chain.bone_count - 1; i++) {
      for (xn::pio::SmoothServo &s : phys_arm.servos[i]) {
        if (s.axis.y > 0)
          continue;
        float fac = s.target_angle - M_PI / 2;
        if (i != 3)
          fac *= -1;
        ac += fac;
      }
    }
    wrist.target_angle = ac + M_PI / 2;

    tot_err +=
        vec3::dist(phys_arm.positions[2], phys_arm.ik_chain.positions[2]);
    avg_err = tot_err / (++iters);

    // render
#ifdef PIO_VIRTUAL
    set_bone_positions(phys_arm);
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

    cam_shader.setVec4("color", glm::vec4(1.0f, 0.5f, 0.2f, 1.0f));
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    for (int j = 0; j < arm.bone_count; j++) {
      ;
      gl::draw_arrays(ideal_joint_positions[j], joint_scale, cam_shader, 36);
      // draw_bone(phys_arm.positions[j], cam_shader, phys_arm.constraints[j],
      // phys_arm.servos[j], true);
    }

    cam_shader.setVec4("color", glm::vec4(1.0f, 0.0f, 0.0f, 1.0f));
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    for (int j = 0; j < arm.bone_count; j++) {
      gl::draw_arrays(phys_joint_positions[j], joint_scale, cam_shader, 36);
      // draw_bone(phys_arm.positions[j], cam_shader, phys_arm.constraints[j],
      // phys_arm.servos[j], true);
    }

    cam_shader.setVec4("color", glm::vec4(0.0f, 1.0f, 0.0f, 1.0f));
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glm::vec3 gpole(pole.x, pole.y, pole.z);
    glm::vec3 pole_scale(0.05, 0.05, 0.05);
    gl::draw_arrays(gpole, pole_scale, cam_shader, 36);

    cam_shader.setVec4("color", glm::vec4(0.0f, 0.0f, 1.0f, 1.0f));
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glm::vec3 gtarget(target.x, target.y, target.z);
    gl::draw_arrays(gtarget, pole_scale, cam_shader, 36);

    gl::draw_grid(VAO_grid, cam_shader);

    glfwSwapBuffers(window);
    glfwPollEvents();
#endif
    time_sleep(0.25);
  }
#ifdef PIO_VIRTUAL
  glDeleteVertexArrays(1, &VAO);
  glDeleteBuffers(1, &VBO);
  glfwTerminate();
#endif
  printf("\n");
  DUMP(avg_err);
}

#ifdef PIO_VIRTUAL
void time_sleep(double seconds) {
  struct timespec ts, rem;

  if (seconds > 0.0) {
    ts.tv_sec = seconds;
    ts.tv_nsec = (seconds - (double)ts.tv_sec) * 1E9;

    while (clock_nanosleep(CLOCK_REALTIME, 0, &ts, &rem)) {
      /* copy remaining time to ts */
      ts.tv_sec = rem.tv_sec;
      ts.tv_nsec = rem.tv_nsec;
    }
  }
}
void draw_bone(vec3 &position, const Shader &shader,
               ik::IkConstraint &constraint, vector<pio::ServoAngular> &servos,
               int isWireFrame = false) {
  if (isWireFrame) {
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  } else {
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  }

  glm::vec3 scale(1, 1, 1);
  glm::vec3 pos(position.x, position.y, position.z);
  glm::mat4 model = glm::mat4(1.0f);
  model = glm::translate(model, pos);
  // model = glm::scale(model, scale);
  for (int i = 0; i < constraint.num_axes; i++) {
    glm::vec3 ax(constraint.axes[i].x, constraint.axes[i].y,
                 constraint.axes[i].z);
    constraint.axes[i].print();
    DUMP(servos[i].getAngle());
    model = glm::rotate(model, servos[i].getAngle(), ax);
  }
  shader.setMat4("model", model);
  glDrawArrays(GL_TRIANGLES, 0, 36);
}
void set_bone_positions(ik::ServoChain arm) {
  for (int i = 0; i < arm.ik_chain.bone_count; i++) {
    vec3 &v = arm.ik_chain.positions[i];
    ideal_joint_positions[i] = glm::vec3(v.x, v.y, v.z);
  }

  for (int i = 0; i < arm.ik_chain.bone_count; i++) {
    vec3 &v = arm.positions[i];
    phys_joint_positions[i] = glm::vec3(v.x, v.y, v.z);
  }
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
#endif