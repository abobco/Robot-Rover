#pragma once

#define PIO_VIRTUAL
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include "app/app_state.hpp"
#include "app/robo_imgui_vulkan.hpp"
#include "app/robo_uniforms_vulkan.hpp"

#include "robo/xn_console.hpp"
#include "robo/xn_pointcloud.hpp"
#include "util/xn_asset_loader.hpp"
#include <algorithm>

namespace xn{
class App{
public:
	static json settings;
	static std::string settings_filepath;
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
}