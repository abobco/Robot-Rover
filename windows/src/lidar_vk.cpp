

#define PIO_VIRTUAL
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include "app/app_state.hpp"
#include "app/robo_imgui_vulkan.hpp"
#include "app/robo_uniforms_vulkan.hpp"

#include "robo/xn_console.hpp"
#include "robo/xn_pointcloud.hpp"
#include "util/xn_asset_loader.hpp"
#include <algorithm>

using namespace xn;

static const std::string DEFAULT_SETTINGS_FILE_PATH =
    "C:/Code/opengl-es/robot-host-app/windows/assets/config/vk_config.json";
std::string settings_filepath = DEFAULT_SETTINGS_FILE_PATH;
json settings;

std::vector<PointCloud> PointCloud::all;
bool mouse_held = false;

void console_update_thread(SshConsole *console);

// graphics mesh data for navigation area
struct NavMesh : public vulkan::Mesh {
  NavMesh(vulkan::Context &ctx, const PointCloud &pc) {
    std::vector<glm::vec3> nv;
    std::vector<vulkan::Vertex> verts;
    std::vector<uint16_t> indices;
    size_t i;
    pc.toNavmesh(nv, glm::vec3(0, 1, 0));
    indices.resize(nv.size());
    verts.resize(nv.size());
    for (i = 0; i < nv.size(); i++)
      verts[i] = {nv[i], glm::vec3(1), glm::vec2(1), glm::vec3(0, 1, 0)};
    for (i = 0; i < nv.size(); i++)
      indices[i] = i;

    vertexBuffer.create(ctx.commandPool.handle, ctx.device, ctx.physicalDevice,
                        verts);
    indexBuffer.create(ctx.commandPool.handle, ctx.device, ctx.physicalDevice,
                       indices);
  }
};

class App {
public:
  glfw::Window window;
  glm::vec4 clear_color{0.02f, 0.02f, 0.02f, 1.00f};
  vulkan::Context ctx;
  std::ofstream debug_file;
  vulkan::SwapChain swapChain;

  vulkan::DevicePair &gpu = ctx.gpu;

  std::vector<std::thread> threads;
  SshConsole *console;
  RobotSettingsWindow settingsWindow;
  vulkan::Image camImage;
  ImTextureID camImageId;

  RobotController *robot;

  Scene scene;
  std::vector<std::vector<glm::vec3>> nav_verts;
  vulkan::Mesh cubeMesh, gridMesh;
  GridGraph navgraph;

  std::map<std::string, vulkan::Image> imageMap;
  std::map<std::string, std::vector<vulkan::UniformCreateInfo>> uniformInfo;
  glm::vec3 cubeRotation = glm::vec3(0);
  UniformLight light{glm::vec3(1), glm::vec3(0), glm::vec3(0.0, -2.0, 1.5)};

  float deltaTime = 0.0f;
  float lastFrame = 0.0f;
  bool orth = false, orbit = true;

  glm::dvec2 cursor = glm::dvec2(0);

  static std::string buildAssetPath(const json &relpath) {
    return (std::string)settings["asset_path"] + "/" + (std::string)relpath;
  }

  App() {}

  App(const std::string &settings_file) {
    settings = load_json_file(settings_file);
    debug_file = std::ofstream((std::string)settings["debug_file"]);
    settingsWindow.setUserCallback(vulkan::drawRenderTreeGui);
  }

  void run() {
    console = new SshConsole(settings);
    robot = new RobotController();
    threads.push_back(std::thread(console_update_thread, console));
    threads.push_back(std::thread(ik_sim_thread, std::ref(robot->arm)));
    threads.push_back(
        std::thread(car_sim_thread, std::ref(navgraph), std::ref(*robot)));

    initVulkan();

    PointCloud::read_pointclouds(buildAssetPath("pointcloud/livingroom.pc"),
                                 PointCloud::all);
    if (PointCloud::all.size() > 0) {
      PointCloud::march_squares(PointCloud::all, 50, 50, navgraph.boxSize,
                                robot->rover.position, navgraph.cells,
                                navgraph.offset);
      navgraph.graph = preprocess_graph(navgraph.cells);
    }

    cubeMesh = vulkan::Mesh(ctx, vulkan::getCubeFaceVerts(),
                            vulkan::getCubeFaceIndices());
    gridMesh = vulkan::Mesh::gen_grid(ctx, navgraph.boxSize, 100, 100);

    createScene();

    mainLoop();
    cleanup();
  }

private:
  void mainLoop() {
    while (!glfwWindowShouldClose(window.handle)) {
      glfwPollEvents();

      if (window.getKey(GLFW_KEY_ESCAPE) == GLFW_PRESS)
        window.close();

      if (!ImGui::GetIO().WantCaptureMouse)
        processInput();

      if (AppState::get().path_needs_update) {
        AppState::get().path_needs_update = false;
        recreatePath();
      } else if (navgraph.path.size() == 0) {
        scene.popChild("path");
        // scene.root.popChild(scene.drawMap["path"]->id);
      }

      drawFrame();
    }
  }

  void updatePointCloudMesh() {
    static vulkan::Mesh scan_mesh;
    static DrawNode *sd = NULL;
    std::vector<glm::vec3> all_points;
    for (const auto &p : PointCloud::all)
      all_points.insert(all_points.begin(), p.points.begin(), p.points.end());
    scan_mesh.vertexBuffer.destroy(ctx.device.handle);
    scan_mesh.indexBuffer.destroy(ctx.device.handle);
    vulkan::Mesh::gen_box_batch(ctx, scan_mesh, all_points, glm::vec3(0.1f));

    if (sd != NULL) {
      delete scene.popChild(sd);
    }
    sd = scene.addChild("color_lit", &scan_mesh,
                        PushConstColor(vulkan::sample_colors[0]));
  }

  void createScene() {
    // assign to drawMap keys for later reference
    scene.addChild("lightSource", "textured", &cubeMesh);
    scene.addChild("cube", "lit", &cubeMesh);

    // updatePointCloudMesh();

    { // add gridbox batch drawnode
      std::vector<glm::vec3> gridverts;
      PointCloud::gridgraph_to_verts(navgraph.cells, gridverts,
                                     navgraph.boxSize, navgraph.offset);
      vulkan::Mesh *gridmesh = new vulkan::Mesh();
      vulkan::Mesh::gen_box_batch(ctx, *gridmesh, gridverts,
                                  glm::vec3(navgraph.boxSize));
      scene.addChild("color_lit", gridmesh,
                     PushConstColor(vulkan::sample_colors[1]));
    }

    // create navmesh
    for (auto &p : PointCloud::all) {
      nav_verts.push_back(std::vector<glm::vec3>());
      std::vector<glm::vec3> &navmesh = nav_verts.back();
      std::vector<vulkan::Vertex> verts;
      std::vector<uint16_t> indices;
      p.toNavmesh(navmesh, glm::vec3(0, 1, 0));
      indices.resize(navmesh.size());
      verts.resize(navmesh.size());
      for (auto i = 0; i < navmesh.size(); i++)
        verts[i] = {navmesh[i], glm::vec3(1), glm::vec2(1), glm::vec3(0, 1, 0)};
      for (auto i = 0; i < navmesh.size(); i++)
        indices[i] = i;
      scene.addChild("color_lit", new vulkan::Mesh(ctx, verts, indices),
                     PushConstColor(glm::vec4(0.8f, 0.6f, 0.4f, 0.5f)));
    }

    // create arm joints
    scene.drawMap["arm"] = scene.root.addChild(new DrawNode());
    vulkan::GraphicsPipeline &gp = scene.pipelineMap["color_lit"];
    AppState::get().arm_mut.lock();
    for (int i = 0; i < robot->arm.joints.numJoints; i++)
      scene.drawMap["arm"]->addChild(
          new DrawNode(&gp, &cubeMesh),
          PushConstColor(robot->arm.joints.positions[i].toGLM(),
                         glm::vec3(0.1f), vulkan::sample_colors[2]));
    AppState::get().arm_mut.unlock();

    scene.drawMap["target"] = scene.drawMap["arm"]->addChild(
        new DrawNode(&gp, &cubeMesh),
        PushConstColor(robot->arm.target.toGLM(), glm::vec3(0.1f),
                       glm::vec4(0.8, 0.2, 0.2, 1.0)));

    // create car mesh
    scene.addChild(
        "car", "color_lit",
        new vulkan::Mesh(ctx, vulkan::getTriVerts(), vulkan::getTriIndices()),
        PushConstColor(glm::vec3(0, 0.1f, 0), glm::vec3(0.25f),
                       glm::vec4(1.0f, 1.0f, 0.2f, 1.0f)));

    // create grid
    float half_ext = navgraph.boxSize * 0.5f;
    scene.addChild("lines", &gridMesh,
                   PushConstColor(navgraph.offset +
                                  glm::vec3(half_ext, 0.001f, half_ext)));

    scene.init(ctx, swapChain, uniformInfo);
    vulkan::gui::init(ctx, window, scene.pipelineMap["lit"], swapChain);
    camImageId = vulkan::ImGuiAddImage(imageMap["brick"]);
  }

  ImDrawData *drawImgui() {
    vulkan::gui::newFrame();

    if (AppState::get().pic_needs_update) {
      // frame_mut.lock();
      AppState::get().frame_mut.lock();
      AppState::get().pic_needs_update = false;
      vulkan::Image newImage;
      int res = 0;

      if (AppState::get().received_pic) {
        // robot->cam_pic_processed.buffer
        res = newImage.fromBuffer(gpu, ctx.commandPool.handle,
                                  robot->cam_outframe);
        // cv::Mat cvtImage;
        // cv::cvtColor(robot->cam_outframe, cvtImage, cv::COLOR_RGB2BGR);
        // res = newImage.fromBufferRaw(gpu, ctx.commandPool.handle,
        //                              cvtImage.ptr(), cvtImage.rows,
        //                              cvtImage.cols, 3,
        //                              VK_FORMAT_R8G8B8_SINT);
      } else
        newImage = imageMap["brick"];
      // frame_mut.unlock();
      if (res == 0) {
        camImage.destroy(gpu.device);
        camImage = newImage;
        camImage.createTextureView(gpu.device);
        camImage.createSampler(gpu);
        camImageId = vulkan::ImGuiAddImage(camImage);
      } else {
        newImage.destroy(gpu.device);
      }

      AppState::get().frame_mut.unlock();
    }
    TextureWindow::draw(camImageId, ImVec2(640, 480), *robot);

    settingsWindow.draw(*robot, (void *)&scene);
    bool t = true;
    console->Draw("Ssh: Console", &t, *robot, navgraph);

    // Rendering
    ImGui::Render();
    return ImGui::GetDrawData();
  }

  void drawFrame() {
    ImDrawData *data = drawImgui();
    uint32_t imageIndex = swapChain.getNextFrame(gpu.device);
    if (imageIndex == -1) {
      recreateSwapChain();
      return;
    }

    createArmMesh();
    updatePointCloudMesh();

    for (auto &keyVal : scene.pipelineMap)
      keyVal.second.setClearColor(clear_color);

    // write command buffer for each frame in swapchain
    for (auto i = 0; i < ctx.commandPool.commandBuffers.size(); i++) {
      ctx.beginDrawCommands(swapChain, i, scene.pipelineMap["textured"]);
      scene.draw(ctx, imageIndex, i);
      ImGui_ImplVulkan_RenderDrawData(data, ctx.commandPool.commandBuffers[i]);
      ctx.endDrawCommands(i);
    }

    updateUniforms(imageIndex);

    if (ctx.draw(swapChain, imageIndex) < 0) {
      recreateSwapChain();
    }
  }

  void initVulkan() {
    window = glfw::Window(settings);
    window.setPositionCallback(window_pos_callback);
    window.setResizeCallback(framebufferResizeCallback, &swapChain);

    ctx.createInstance(debugWriteToFileCallback, (void *)&debug_file);
    ctx.createDevices(window);
    ctx.createCommandPool();
    gpu.physicalDevice.print();

    swapChain.create(gpu, window);
    swapChain.createImageViews(gpu.device);

    // load images
    for (const auto &[key, value] : settings["textures"].items()) {
      imageMap[key] =
          vulkan::Image(gpu, ctx.commandPool.handle, buildAssetPath(value))
              .createTextureView(gpu.device)
              .createSampler(gpu);
    }

    // create grapihcs pipelines
    auto shaderMap =
        AssetLoader::load_shaders(settings["asset_path"], settings["shaders"]);
    shaderMap["lines"] = shaderMap["color"];
    for (auto &[key, val] : shaderMap)
      scene.addPipeline(key, val.first, val.second);
    scene.pipelineMap["lines"].polygonMode = VK_POLYGON_MODE_LINE;
    scene.pipelineMap["lines"].topologyType = VK_PRIMITIVE_TOPOLOGY_LINE_LIST;
    for (auto &[key, val] : scene.pipelineMap)
      val.createPushConstantBuffer(gpu.physicalDevice);
    std::vector<std::string> colorPipelines = {"lines", "color", "color_lit"};
    scene.setPushConstants(colorPipelines, PushConstColor());

    // create uniforms
    vulkan::UniformCreateInfo mvpInfo("mvp", sizeof(UniformMvp), 0);
    vulkan::UniformCreateInfo imageInfo(&imageMap["brick"], 1);
    vulkan::UniformCreateInfo lightInfo("light", sizeof(UniformLight), 2,
                                        VK_SHADER_STAGE_FRAGMENT_BIT);
    uniformInfo["textured"] = {mvpInfo, imageInfo};
    uniformInfo["lines"] = {mvpInfo};
    uniformInfo["lit"] = {mvpInfo, imageInfo, lightInfo};
    uniformInfo["color"] = {mvpInfo};
    lightInfo.binding = 1;
    uniformInfo["color_lit"] = {mvpInfo, lightInfo};
  }

  void updateUniforms(uint32_t imageIndex) {
    light.viewPos = scene.mainCamera.pos;
    light.color = glm::vec3(1.0f);
    scene.setUniform("lit", "light", light, imageIndex, gpu.device);
    scene.setUniform("color_lit", "light", light, imageIndex, gpu.device);

    glm::mat4 model = transform_model(light.pos, glm::vec3(0.1f));
    scene.drawMap["lightSource"]->setModel(model);

    model = transform_model(glm::vec3(5, 5, 0));
    glm::vec3 rotScaled = cubeRotation * glm::radians(360.0f);
    model = rotate_eulers(rotScaled, model);
    scene.drawMap["cube"]->setModel(model);

    glm::vec3 car_pos_3d(robot->rover.position.x, 0.1, robot->rover.position.y);
    float car_ang_corrected = (float)-M_PI_2 + robot->rover.rotation;
    model = transform_model(car_pos_3d, glm::vec3(0.25));
    model = rotate_eulers(glm::vec3(0, car_ang_corrected, 0), model);
    PushConstColor pcc{model, glm::vec4(1.0f, 1.0f, 0.2f, 1.0f)};
    scene.drawMap["car"]->setPushConstant(pcc);
    model = transform_model(car_pos_3d);
    model = rotate_eulers(glm::vec3(0, car_ang_corrected, 0), model);
    model = transform_model(robot->arm.target.toGLM(), glm::vec3(0.1f), model);
    scene.drawMap["target"]->setPushConstant(
        PushConstColor(model, glm::vec4(0.8, 0.2, 0.2, 1.0)));

    for (auto i = 0; i < robot->arm.joints.numJoints; i++) {
      model = transform_model(car_pos_3d);
      model = rotate_eulers(glm::vec3(0, car_ang_corrected, 0), model);
      model = transform_model(robot->arm.joints.positions[i].toGLM(),
                              glm::vec3(0.1f), model);
      scene.drawMap["arm"]->children[i]->setPushConstant(
          PushConstColor(model, vulkan::sample_colors[2]));
    }
  }

  void recreateSwapChain() {
    while (window.isMinimized())
      glfwWaitEvents();
    vkDeviceWaitIdle(gpu.device.handle);

    ctx.commandPool.destroy(gpu.device);
    swapChain.recreate(gpu, window);
    scene.recreatePipelines(ctx, swapChain, uniformInfo);
    swapChain.createFrameResources(gpu, ctx.commandPool,
                                   scene.pipelineMap["textured"].renderPass);
    ctx.createCommandBuffers(swapChain);
    swapChain.imagesInFlight.resize(swapChain.images.size(), VK_NULL_HANDLE);
    ImGui_ImplVulkan_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    vulkan::gui::init(ctx, window, scene.pipelineMap["textured"], swapChain);
    camImageId = vulkan::ImGuiAddImage(imageMap["brick"]);

    scene.mainCamera.projection = swapChain.getProjectionMatrix();
  }

  void cleanup() {
    cubeMesh.destroy(ctx);

    // ctx.cleanupSwapChain(swapChain);
    swapChain.destroy(gpu.device);
    ctx.cleanup(swapChain);

    vkDestroySurfaceKHR(ctx.instance, window.surface, nullptr);
    vkDestroyInstance(ctx.instance, nullptr);
    glfwDestroyWindow(window.handle);
    glfwTerminate();
    debug_file.close();

    AppState::get().run = false;
    for (auto &t : threads) {
      t.join();
    }
  }

  void update_path(GLFWwindow *window) {
    glfwGetCursorPos(window, &cursor.x, &cursor.y);
    glm::ivec2 winsize;
    glfwGetWindowSize(window, &winsize.x, &winsize.y);
    // xn_printf("cursor: (%.3f, %.3f)\n", cursor.x, cursor.y);

    if (navgraph.graph.size() > 0 && AppState::get().path_mut.try_lock()) {
      glm::ivec2 target;
      glm::vec3 start_world(robot->rover.position.x, 0,
                            robot->rover.position.y);
      glm::ivec2 start =
          world_to_grid(start_world, navgraph.boxSize, navgraph.offset);
      Plane p{0, {0, 1, 0}};
      glm::vec3 ray_a, ray_b, q;
      scene.mainCamera.get_pixelray(cursor, winsize, ray_a, ray_b);
      float t = 0;
      if (collision_segment_plane(ray_a, ray_b, p, t, q)) {
        robot->rover.target = q;
        target = world_to_grid(q, navgraph.boxSize, navgraph.offset);
      }
      target.x = clamp(target.x, 0, (int)navgraph.cells.size());
      target.y = clamp(target.y, 0, (int)navgraph.cells.front().size());
      start.x = clamp(start.x, 0, (int)navgraph.cells.size());
      start.y = clamp(start.y, 0, (int)navgraph.cells.front().size());
      navgraph.path =
          A_star(navgraph.graph, start.x, start.y, target.x, target.y);
      AppState::get().path_mut.unlock();
    }
  }

  void processInput() {
    float currentFrame = (float)glfwGetTime();
    deltaTime = currentFrame - lastFrame;
    lastFrame = currentFrame;
    scene.mainCamera.handleInput(window, deltaTime);

    int mouse_state = glfwGetMouseButton(window.handle, GLFW_MOUSE_BUTTON_LEFT);
    if (mouse_state == GLFW_PRESS) {
      update_path(window.handle);
      recreatePath();
    }

    static bool c_pressed = false, f_pressed = false;
    // toggle top-down orthographic view
    bool c_pressed_prev = c_pressed;
    window.handleToggle(GLFW_KEY_C, c_pressed, orth);
    if (c_pressed_prev != c_pressed) {
      scene.mainCamera.rad = 5;
    }

    if (orth) {
      scene.mainCamera.type = CAMERA_TOPDOWN;
      scene.mainCamera.up = glm::vec3(0, 0, 1);
      return;
    }

    scene.mainCamera.up = glm::vec3(0, 1, 0);

    bool orbit_prev = orbit;
    window.handleToggle(GLFW_KEY_F, f_pressed, orbit);
    if (orbit) {
      if (!orbit_prev) {
        glfwSetInputMode(window.handle, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
      }
      scene.mainCamera.type = CAMERA_ORBIT;
      scene.mainCamera.target = glm::vec3(0);
    } else {
      if (orbit_prev) {
        glfwSetInputMode(window.handle, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
        if (glfwRawMouseMotionSupported())
          glfwSetInputMode(window.handle, GLFW_RAW_MOUSE_MOTION, GLFW_TRUE);
      }
      scene.mainCamera.type = CAMERA_FLY;
    }
  }

  void recreatePath() {
    if (AppState::get().path_mut.try_lock()) {
      if (scene.drawMap.find("path") != scene.drawMap.end()) {
        delete scene.popChild("path");
      }
      scene.drawMap["path"] = scene.root.addChild(new DrawNode());

      vulkan::GraphicsPipeline &gp = scene.pipelineMap["color_lit"];
      for (auto i = 0; i < navgraph.path.size(); i++) {
        DrawNode *n =
            scene.drawMap["path"]->addChild(new DrawNode(&gp, &cubeMesh));
        glm::mat4 model =
            transform_model(navgraph.offset, glm::vec3(navgraph.boxSize));
        model = glm::translate(
            model, glm::vec3(navgraph.path[i].x, -0.25, navgraph.path[i].y));

        glm::vec4 color;
        if (i == 0)
          color = glm::vec4(0, 0, 0.8, 1.0);
        else if (i == navgraph.path.size() - 1)
          color = glm::vec4(0.8, 0, 0, 1.0);
        else
          color = glm::vec4(0, 0.8, 0, 1.0);
        n->setPushConstant(PushConstColor{model, color});
      }

      AppState::get().path_mut.unlock();
    } else {
      // AppState::get().path_needs_update = true;
    }
  }

  void createArmMesh() {
    int n = robot->arm.joints.numJoints;
    glm::vec3 arm_color(0.2, 0.2, 0.9);
    std::vector<vulkan::Vertex> arm_verts(n);
    std::vector<uint16_t> arm_indices;
    glm::vec3 car_pos_3d(robot->rover.position.x, 0.1, robot->rover.position.y);
    float car_ang_corrected = (float)-M_PI_2 + robot->rover.rotation;
    for (auto i = 0; i < n; i++) {
      glm::vec3 pos = robot->arm.joints.positions[i].toGLM();
      glm::mat4 model = transform_model(car_pos_3d);
      model = rotate_eulers(glm::vec3(0, car_ang_corrected, 0), model);
      model = transform_model(robot->arm.joints.positions[i].toGLM(),
                              glm::vec3(1), model);
      arm_verts[i] = {model * glm::vec4(0, 0, 0, 1), arm_color, glm::vec2(1),
                      glm::vec3(0, 1, 0)};
      arm_indices.push_back(i);
      if (i != 0 && i != (n - 1)) {
        arm_indices.push_back(i);
      }
    }

    auto skel_pos = scene.drawMap.find("arm_skeleton");
    if (skel_pos != scene.drawMap.end()) {
      DrawNode *d = scene.drawMap["arm"]->popChild(skel_pos->second->id);
      for (auto m : d->meshes) {
        m->vertexBuffer.destroy(gpu.device.handle);
        m->indexBuffer.destroy(gpu.device.handle);
        delete m;
      }
      delete d;
    }
    scene.drawMap["arm_skeleton"] = scene.drawMap["arm"]->addChild(
        new DrawNode(&scene.pipelineMap["lines"],
                     new vulkan::Mesh(ctx, arm_verts, arm_indices)),
        PushConstColor(glm::vec4(arm_color, 1.0)));
  }

  static VkBool32 debugWriteToFileCallback(
      VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity,
      VkDebugUtilsMessageTypeFlagsEXT messageType,
      const VkDebugUtilsMessengerCallbackDataEXT *pCallbackData,
      void *pUserData) {
    // write validation errors to debug file
    std::ofstream *debug_file = (std::ofstream *)pUserData;
    *debug_file << "[validation layer] " << pCallbackData->pMessage
                << std::endl;
    return VK_FALSE;
  }

  static void framebufferResizeCallback(GLFWwindow *window, int width,
                                        int height) {
    auto swapChain =
        reinterpret_cast<vulkan::SwapChain *>(glfwGetWindowUserPointer(window));
    swapChain->framebufferResized = true;
    settings["resolution"][0] = width;
    settings["resolution"][1] = height;
    std::fstream f(settings_filepath);
    f << std::setw(4) << settings << std::endl;
    f.close();
  }

  static void window_pos_callback(GLFWwindow *window, int xpos, int ypos) {
    settings["window_position"][0] = xpos;
    settings["window_position"][1] = ypos;
    std::fstream f(settings_filepath);
    f << std::setw(4) << settings << std::endl;
    f.close();
  }
};

int main(int argc, char **argv) {
  if (argc > 1)
    settings_filepath = argv[1];

  App app(settings_filepath);
  try {
    app.run();
  } catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}

void console_update_thread(SshConsole *console) {
  StreamHook cout_hook(std::cout), cerr_hook(std::cerr);
  while (AppState::get().run) {
    std::string contents;
    if (console->istream.tellp() > 0) {
      contents = console->istream.str();
      console->AddLog("%s", contents.c_str());
      console->istream.str(std::string());
    }
    while (cout_hook.bytesAvailable()) {
      cout_hook.dump(contents);
      console->AddLog("# %s", contents.c_str());
    }
    while (cerr_hook.bytesAvailable()) {
      cerr_hook.dump(contents);
      console->AddLog("[error] %s", contents.c_str());
    }
    time_sleep(0.1);
  }
}