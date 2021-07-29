

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

class App{
public:
	glfw::Window window;
	glm::vec4 clear_color{0.02f, 0.02f, 0.02f, 1.00f};
	vulkan::Context ctx;
	vulkan::SwapChain swapChain;
	vulkan::Image camImage;

	RobotController* robot = NULL;
	GridGraph navgraph;

	SshConsole* console = NULL;
	RobotSettingsWindow settingsWindow;
	CamWindow camWindow;

	Scene scene;
	vulkan::Mesh cubeMesh, gridMesh, triMesh;

	static std::string assetPath(const json& relpath);

	App(){}

	App(const std::string& settings_file);

	void run();

private:
	void createScene();

	void mainLoop();

	void updateBoxBatch(std::vector<glm::vec3>& points, vulkan::Mesh& mesh,
		DrawNode& node, const glm::vec3& scale = glm::vec3(0.05),
		const glm::vec4& color = vulkan::sample_colors[0]);

	void updatePointCloudMesh();

	ImDrawData* drawImgui();

	void drawFrame();

	void initVulkan();

	void updateUniforms(uint32_t imageIndex);

	void recreateSwapChain();

	void cleanup();

	void update_path(GLFWwindow* window);

	void processInput();

	void recreatePath();

	void createArmMesh();

	static void framebufferResizeCallback(GLFWwindow* window, int width, int height);

	static void window_pos_callback(GLFWwindow* window, int xpos, int ypos);
};

int main(int argc, char** argv){
	if (argc > 1)
		settings_filepath = argv[1];
	App app(settings_filepath);
	try{
		app.run();
	}
	catch (const std::exception& e){
		std::cerr << e.what() << std::endl;
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
}

inline std::string App::assetPath(const json& relpath){
	return (std::string)settings["asset_path"] + "/" + (std::string)relpath;
}

inline App::App(const std::string& settings_file){
	settings = load_json_file(settings_file);
	settingsWindow.setUserCallback(vulkan::drawRenderTreeGui);

	console = new SshConsole(settings);
	robot = new RobotController();
}

inline void App::run(){
	initVulkan();
	createScene();

	std::vector<std::thread> threads;
	threads.push_back(std::thread(SshConsole::update_thread, console));
	threads.push_back(std::thread(ik_sim_thread, std::ref(robot->arm)));
	threads.push_back(std::thread(car_sim_thread, std::ref(navgraph), std::ref(*robot)));

	mainLoop();

	cleanup();

	AppState::get().run = false;
	for (auto& t : threads){
		t.join();
	}
}

inline void App::createScene(){
	ModelMatrix model;

	// load point cloud file
	{
		PointCloud::read_pointclouds(assetPath("pointcloud/livingroom.pc"),
			PointCloud::all);
		AppState::get().dirty_points = true;

		if (PointCloud::all.size() > 0){
			PointCloud::march_squares(PointCloud::all, 50, 50, navgraph.boxSize,
				robot->rover.position, navgraph.cells,
				navgraph.offset);
			navgraph.graph = preprocess_graph(navgraph.cells);
		}
		AppState::get().scan_needs_update = true;
	}

	// create meshes
	{
		cubeMesh = vulkan::Mesh(ctx, vulkan::getCubeFaceVerts(), vulkan::getCubeFaceIndices());
		gridMesh = vulkan::Mesh::gen_grid(ctx, navgraph.boxSize, 100, 100);
		triMesh = vulkan::Mesh(ctx, vulkan::getTriVerts(), vulkan::getTriIndices());
		scene.addChild("lightSource", "textured", &cubeMesh);
	}

	// create arm joints
	{
		scene.drawMap["arm"] = scene.root.addChild(new DrawNode());
		vulkan::GraphicsPipeline& gp = scene.pipelineMap["color_lit"];
		for (int i = 0; i < robot->arm.joints.numJoints; i++){
			model.toIdentity()
				.translate(robot->arm.joints.positions[i].toGLM())
				.scale(0.1f);

			scene.drawMap["arm"]->addChild(
				new DrawNode(&gp, &cubeMesh),
				PushConstColor(model, vulkan::sample_colors[2]));
		}

		model.toIdentity()
			.translate(robot->arm.target.toGLM())
			.scale(0.1f);
		scene.drawMap["target"] = scene.drawMap["arm"]->addChild(
			new DrawNode(&gp, &cubeMesh),
			PushConstColor(model, glm::vec4(0.8, 0.2, 0.2, 1.0)));
	}

	// create car mesh
	{
		model.toIdentity()
			.translate(glm::vec3(0, 0.1f, 0))
			.scale(0.25f);
		scene.addChild("car", "color_lit", &triMesh,
			PushConstColor(model, glm::vec4(1.0f, 1.0f, 0.2f, 1.0f)));
	}

	// create grid
	{
		float half_ext = navgraph.boxSize * 0.5f;
		if (PointCloud::all.size() == 0)
			navgraph.offset = {0, 0, 0};

		model.toIdentity()
			.translate(navgraph.offset + glm::vec3(half_ext, 0.001f, half_ext));
		scene.addChild("grid", "lines", &gridMesh, PushConstColor(model));
	}

	// init scene
	{
		scene.init(ctx, swapChain);
		vulkan::gui::init(ctx, window, scene.pipelineMap["lit"], swapChain);
		camWindow.id = vulkan::ImGuiAddImage(scene.imageMap["brick"]);
	}
}

inline void App::mainLoop(){
	while (!glfwWindowShouldClose(window.handle)){
		glfwPollEvents();

		if (window.getKey(GLFW_KEY_ESCAPE) == GLFW_PRESS)
			window.close();

		if (!ImGui::GetIO().WantCaptureMouse)
			processInput();

		if (AppState::get().path_needs_update){
			AppState::get().path_needs_update = false;
			recreatePath();
		} else if (navgraph.path.size() == 0){
			scene.popChild("path");
		}

		drawFrame();
	}
}

inline void App::updateBoxBatch(std::vector<glm::vec3>& points, vulkan::Mesh& mesh, DrawNode& node, const glm::vec3& scale, const glm::vec4& color){
	if (points.size() == 0)
		return;

	if (node.empty()){
		node = DrawNode(&scene.pipelineMap["color_lit"], &mesh);
		scene.root.addChild(&node, PushConstColor(color));
	}

	mesh.destroy(ctx);
	vulkan::Mesh::gen_box_batch(ctx, mesh, points, scale);
}

inline void App::updatePointCloudMesh(){
	static vulkan::Mesh scan_mesh, gridmesh, navmesh;
	static DrawNode sd, gridnode, navnode;
	std::vector<glm::vec3> all_points;
	std::vector<glm::vec3> gridverts;

	// update pointcloud boxes
	if (AppState::get().dirty_points){
		AppState::get().dirty_points = false;
		PointCloud::combine_points(PointCloud::all, all_points);
		updateBoxBatch(all_points, scan_mesh, sd, glm::vec3(0.025f));
	}

	// add gridbox batch drawnode
	if (AppState::get().scan_needs_update){
		AppState::get().scan_needs_update = false;
		navgraph.toVerts(gridverts);
		updateBoxBatch(gridverts, gridmesh, gridnode, glm::vec3(navgraph.boxSize),
			vulkan::sample_colors[1]);

		// create navmesh
		for (auto& p : PointCloud::all){
			std::vector<glm::vec3> nav_verts;
			p.toNavmesh(nav_verts, glm::vec3(0, 1, 0));
			navmesh = vulkan::Mesh(ctx, nav_verts);
			if (navnode.empty()){
				navnode = DrawNode(&scene.pipelineMap["color_lit"], &navmesh);
				scene.root.addChild(
					&navnode, PushConstColor(glm::vec4(0.8f, 0.6f, 0.4f, 0.5f)));
			}
		}

		float half_ext = navgraph.boxSize * 0.5f;
		scene.drawMap["grid"]->setPushConst(PushConstColor(
			navgraph.offset + glm::vec3(half_ext, 0.001f, half_ext)));
	}
}

inline ImDrawData* App::drawImgui(){
	vulkan::gui::newFrame();

	if (AppState::get().pic_needs_update){
		AppState::get().frame_mut.lock();
		AppState::get().pic_needs_update = false;
		vulkan::Image newImage;
		int res = 0;

		if (AppState::get().received_pic){
			res = newImage.fromBuffer(ctx.gpu, ctx.commandPool.handle,
				robot->cam_outframe);
		} else
			newImage = scene.imageMap["brick"];
		if (res == 0){
			camImage.destroy(ctx.gpu.device);
			camImage = newImage
				.createTextureView(ctx.gpu.device)
				.createSampler(ctx.gpu);
			camWindow.id = vulkan::ImGuiAddImage(camImage);
		} else{
			newImage.destroy(ctx.gpu.device);
		}

		AppState::get().frame_mut.unlock();
	}
	camWindow.draw(ImVec2(640, 480), *robot);

	settingsWindow.draw(*robot, (void*)&scene);
	bool t = true;
	console->Draw("Ssh: Console", &t, *robot, navgraph);

	// Rendering
	ImGui::Render();
	return ImGui::GetDrawData();
}

inline void App::drawFrame(){
	ImDrawData* data = drawImgui();
	uint32_t imageIndex = swapChain.getNextFrame(ctx.gpu.device);
	if (imageIndex == -1){
		recreateSwapChain();
		return;
	}

	createArmMesh();
	updatePointCloudMesh();

	for (auto& keyVal : scene.pipelineMap)
		keyVal.second.setClearColor(clear_color);

	// write command buffer for each frame in swapchain
	for (auto i = 0; i < ctx.commandPool.commandBuffers.size(); i++){
		ctx.beginDrawCommands(swapChain, i, scene.pipelineMap["textured"]);
		scene.draw(ctx, imageIndex, i);
		ImGui_ImplVulkan_RenderDrawData(data, ctx.commandPool.commandBuffers[i]);
		ctx.endDrawCommands(i);
	}

	updateUniforms(imageIndex);

	if (ctx.draw(swapChain, imageIndex) < 0){
		recreateSwapChain();
	}
}

inline void App::initVulkan(){
	window = glfw::Window(settings);
	window.setPositionCallback(window_pos_callback);
	window.setResizeCallback(framebufferResizeCallback, &swapChain);

	ctx.createInstance(settings["debug_file"]);
	ctx.createDevices(window);
	ctx.createCommandPool();
	ctx.gpu.physicalDevice.print();

	swapChain.create(ctx.gpu, window);
	swapChain.createImageViews(ctx.gpu.device);

	// load images
	for (const auto& [key, value] : settings["textures"].items()){
		scene.imageMap[key] =
			vulkan::Image(ctx.gpu, ctx.commandPool.handle, assetPath(value))
			.createTextureView(ctx.gpu.device)
			.createSampler(ctx.gpu);
	}

	scene.loadPipelines(settings);

	// create push constants
	scene.createPushConstBuffers(ctx);
	scene.setPushConstants({"lines", "color", "color_lit"}, PushConstColor());

	// create uniforms
	vulkan::UniformCreateInfo mvpInfo("mvp", sizeof(UniformMvp));
	vulkan::UniformCreateInfo imageInfo(&scene.imageMap["brick"]);
	vulkan::UniformCreateInfo lightInfo("light", sizeof(UniformLight),
		VK_SHADER_STAGE_FRAGMENT_BIT);

	scene.pipelineMap["textured"]
		.addUniform(mvpInfo)
		.addUniform(imageInfo);
	scene.pipelineMap["lines"]
		.addUniform(mvpInfo);
	scene.pipelineMap["lit"]
		.addUniform(mvpInfo)
		.addUniform(imageInfo)
		.addUniform(lightInfo);
	scene.pipelineMap["color"]
		.addUniform(mvpInfo);
	scene.pipelineMap["color_lit"]
		.addUniform(mvpInfo)
		.addUniform(lightInfo);

	//uniformInfo["textured"] = {mvpInfo, imageInfo};
	//uniformInfo["lines"] = {mvpInfo};
	//uniformInfo["lit"] = {mvpInfo, imageInfo, lightInfo};
	//uniformInfo["color"] = {mvpInfo};
	//lightInfo.binding = 1;
	//uniformInfo["color_lit"] = {mvpInfo, lightInfo};
}

inline void App::updateUniforms(uint32_t imageIndex){
	ModelMatrix model;
	UniformLight light;

	light.color = glm::vec3(1);
	light.pos = glm::vec3(0.0f, -2.0f, 1.5f);
	light.viewPos = scene.mainCamera.pos;
	scene.setUniform("lit", "light", light, imageIndex, ctx.gpu.device);
	scene.setUniform("color_lit", "light", light, imageIndex, ctx.gpu.device);

	model.toIdentity()
		.translate(light.pos)
		.scale(0.1f);
	scene.drawMap["lightSource"]->setModel(model);

	glm::vec3 car_pos_3d(robot->rover.position.x, 0.1, robot->rover.position.y);
	float car_ang_corrected = (float)-M_PI_2 + robot->rover.rotation;

	model.toIdentity()
		.translate(car_pos_3d)
		.scale(0.25f)
		.rotateEulers(glm::vec3(0, car_ang_corrected, 0));
	scene.drawMap["car"]->setPushConst(
		PushConstColor(model, glm::vec4(1.0f, 1.0f, 0.2f, 1.0f)));

	model.toIdentity()
		.translate(car_pos_3d)
		.rotateEulers(glm::vec3(0, car_ang_corrected, 0))
		.translate(robot->arm.target.toGLM())
		.scale(0.1f);
	scene.drawMap["target"]->setPushConst(
		PushConstColor(model, glm::vec4(0.8, 0.2, 0.2, 1.0)));

	for (auto i = 0; i < robot->arm.joints.numJoints; i++){
		model.toIdentity()
			.translate(car_pos_3d)
			.rotateEulers(glm::vec3(0, car_ang_corrected, 0))
			.translate(robot->arm.joints.positions[i].toGLM())
			.scale(0.1f);
		scene.drawMap["arm"]->children[i]->setPushConst(
			PushConstColor(model, vulkan::sample_colors[2]));
	}
}

inline void App::recreateSwapChain(){
	while (window.isMinimized())
		glfwWaitEvents();
	vkDeviceWaitIdle(ctx.gpu.device.handle);

	ctx.commandPool.destroy(ctx.gpu.device);
	swapChain.recreate(ctx.gpu, window);
	scene.recreatePipelines(ctx, swapChain);
	swapChain.createFrameResources(ctx.gpu, ctx.commandPool,
		scene.pipelineMap["textured"].renderPass);
	ctx.createCommandBuffers(swapChain);
	swapChain.imagesInFlight.resize(swapChain.images.size(), VK_NULL_HANDLE);
	ImGui_ImplVulkan_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();
	vulkan::gui::init(ctx, window, scene.pipelineMap["textured"], swapChain);
	camWindow.id = vulkan::ImGuiAddImage(scene.imageMap["brick"]);

	scene.mainCamera.projection = swapChain.getProjectionMatrix();
}

inline void App::cleanup(){
	cubeMesh.destroy(ctx);

	// ctx.cleanupSwapChain(swapChain);
	swapChain.destroy(ctx.gpu.device);
	ctx.cleanup(swapChain);

	vkDestroySurfaceKHR(ctx.instance, window.surface, nullptr);
	vkDestroyInstance(ctx.instance, nullptr);
	glfwDestroyWindow(window.handle);
	glfwTerminate();
	ctx.debug_file.close();
}

inline void App::update_path(GLFWwindow* window){
	glm::dvec2 cursor;
	glm::ivec2 winsize;
	glfwGetCursorPos(window, &cursor.x, &cursor.y);
	glfwGetWindowSize(window, &winsize.x, &winsize.y);

	if (navgraph.graph.size() > 0 && AppState::get().path_mut.try_lock()){
		Plane p{0,{0, 1, 0}};
		glm::vec3 ray_a, ray_b, q;
		scene.mainCamera.get_pixelray(cursor, winsize, ray_a, ray_b);
		float t = 0;
		if (collision_segment_plane(ray_a, ray_b, p, t, q)){
			glm::ivec2 graph_dims(navgraph.colCount(), navgraph.rowCount());
			glm::vec3 start_world(robot->rover.position.x, 0, robot->rover.position.y);
			glm::ivec2 start = world_to_grid(start_world, navgraph.boxSize, navgraph.offset);
			glm::ivec2 target = world_to_grid(q, navgraph.boxSize, navgraph.offset);
			target = clamp_vec2(target, glm::ivec2(0), graph_dims);
			start = clamp_vec2(start, glm::ivec2(0), graph_dims);
			navgraph.path = A_star(navgraph.graph, start.x, start.y, target.x, target.y);
			robot->rover.target = q;
		}
		AppState::get().path_mut.unlock();
	}
}

inline void App::processInput(){
	static float deltaTime = 0.0f;
	static float lastFrame = 0.0f;
	static bool orth = false;
	static bool orbit = true;

	float currentFrame = (float)glfwGetTime();
	deltaTime = currentFrame - lastFrame;
	lastFrame = currentFrame;
	scene.mainCamera.handleInput(window, deltaTime);

	int mouse_state = glfwGetMouseButton(window.handle, GLFW_MOUSE_BUTTON_LEFT);
	if (mouse_state == GLFW_PRESS){
		update_path(window.handle);
		recreatePath();
	}

	static bool c_pressed = false, f_pressed = false;
	// toggle top-down orthographic view
	bool c_pressed_prev = c_pressed;
	window.handleToggle(GLFW_KEY_C, c_pressed, orth);
	if (c_pressed_prev != c_pressed){
		scene.mainCamera.rad = 5;
	}

	if (orth){
		scene.mainCamera.type = CAMERA_TOPDOWN;
		scene.mainCamera.up = glm::vec3(0, 0, 1);
		return;
	}

	scene.mainCamera.up = glm::vec3(0, 1, 0);

	bool orbit_prev = orbit;
	window.handleToggle(GLFW_KEY_F, f_pressed, orbit);
	if (orbit){
		if (!orbit_prev){
			glfwSetInputMode(window.handle, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
		}
		scene.mainCamera.type = CAMERA_ORBIT;
		scene.mainCamera.target = glm::vec3(0);
	} else{
		if (orbit_prev){
			glfwSetInputMode(window.handle, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
			if (glfwRawMouseMotionSupported())
				glfwSetInputMode(window.handle, GLFW_RAW_MOUSE_MOTION, GLFW_TRUE);
		}
		scene.mainCamera.type = CAMERA_FLY;
	}
}

inline void App::recreatePath(){
	if (AppState::get().path_mut.try_lock()){
		if (scene.drawMap.find("path") != scene.drawMap.end()){
			delete scene.popChild("path");
		}
		scene.drawMap["path"] = scene.root.addChild(new DrawNode());

		vulkan::GraphicsPipeline& gp = scene.pipelineMap["color_lit"];
		for (auto i = 0; i < navgraph.path.size(); i++){
			DrawNode* n =
				scene.drawMap["path"]->addChild(new DrawNode(&gp, &cubeMesh));
			ModelMatrix model;
			model.translate(navgraph.offset)
				.scale(navgraph.boxSize)
				.translate(glm::vec3(navgraph.path[i].x, -0.25, navgraph.path[i].y));

			glm::vec4 color;
			if (i == 0)
				color = glm::vec4(0, 0, 0.8, 1.0);
			else if (i == navgraph.path.size() - 1)
				color = glm::vec4(0.8, 0, 0, 1.0);
			else
				color = glm::vec4(0, 0.8, 0, 1.0);
			n->setPushConst(PushConstColor(model, color));
		}

		AppState::get().path_mut.unlock();
	} else{
		// AppState::get().path_needs_update = true;
	}
}

inline void App::createArmMesh(){
	static DrawNode d;
	static vulkan::Mesh mesh;

	ModelMatrix model;
	int n = robot->arm.joints.numJoints;
	glm::vec3 arm_color(0.2, 0.2, 0.9);
	std::vector<vulkan::Vertex> arm_verts(n);
	std::vector<uint16_t> arm_indices;
	glm::vec3 car_pos_3d(robot->rover.position.x, 0.1, robot->rover.position.y);
	float car_ang_corrected = (float)-M_PI_2 + robot->rover.rotation;

	for (auto i = 0; i < n; i++){
		model.toIdentity()
			.translate(car_pos_3d)
			.rotateEulers(glm::vec3(0, car_ang_corrected, 0))
			.translate(robot->arm.joints.positions[i].toGLM());

		arm_verts[i] = vulkan::Vertex{
			model.mat * glm::vec4(0, 0, 0, 1),
			arm_color,
			glm::vec2(1),
			glm::vec3(0, 1, 0)
		};
		arm_indices.push_back(i);
		if (i != 0 && i != (n - 1)){
			arm_indices.push_back(i);
		}
	}

	mesh.destroy(ctx);
	mesh = vulkan::Mesh(ctx, arm_verts, arm_indices);
	if (d.empty()){
		d = DrawNode(&scene.pipelineMap["lines"], &mesh);
		scene.drawMap["arm"]->addChild(&d,
			PushConstColor(glm::vec4(arm_color, 1.0)));
	}
}

inline void App::framebufferResizeCallback(GLFWwindow* window, int width, int height){
	auto swapChain =
		reinterpret_cast<vulkan::SwapChain*>(glfwGetWindowUserPointer(window));
	swapChain->framebufferResized = true;
	settings["resolution"][0] = width;
	settings["resolution"][1] = height;
	std::fstream f(settings_filepath);
	f << std::setw(4) << settings << std::endl;
	f.close();
}

inline void App::window_pos_callback(GLFWwindow* window, int xpos, int ypos){
	settings["window_position"][0] = xpos;
	settings["window_position"][1] = ypos;
	std::fstream f(settings_filepath);
	f << std::setw(4) << settings << std::endl;
	f.close();
}
