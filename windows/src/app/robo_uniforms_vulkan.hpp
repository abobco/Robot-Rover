#pragma once
#include "app_state.hpp"
#include <graphics/vulkan/xn_vulkan.hpp>
#include <graphics/vulkan/xn_vulkan_renderer.hpp>

namespace xn{

struct UniformLight{
	alignas(16) glm::vec3 color;
	alignas(16) glm::vec3 viewPos;
	alignas(16) glm::vec3 pos;
};

struct PushConstColor{
	glm::mat4 model = glm::mat4(1);
	glm::vec4 color = glm::vec4(1);

	PushConstColor(){}

	PushConstColor(glm::vec4 color) : model(glm::mat4(1)), color(color){}
	PushConstColor(glm::mat4 model) : model(model), color(glm::vec4(1)){}

	PushConstColor(glm::mat4 model, glm::vec4 color)
		: model(model), color(color){}

	PushConstColor(ModelMatrix model, glm::vec4 color)
		: model(model.mat), color(color){}

	PushConstColor(glm::vec3 translation, glm::vec3 scale = glm::vec3(1),
		glm::vec4 color = glm::vec4(1)){
		model = glm::translate(glm::mat4(1), translation);
		model = glm::scale(model, scale);
		this->color = color;
	}
};

} // namespace xn