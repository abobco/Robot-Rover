
#include "json.hpp"
#include <fstream>
// #include <graphics/vulkan/core/xn_vk_image.hpp>
#include <map>
#include <utility>
#include <vector>

using json = nlohmann::json;

namespace xn{
//pipelineMap[key].polygonMode = VK_POLYGON_MODE_LINE;
//pipelineMap[key].topologyType = VK_PRIMITIVE_TOPOLOGY_LINE_LIST;

struct ShaderSpirV{
	std::vector<char> vert;
	std::vector<char> frag;
	int64_t polygonMode;
	int64_t topologyType;


	static int64_t getTypeFromString(const std::string& type){
#define vk_str_case(var, val) if (var == #val) return (int64_t)val;
		vk_str_case(type, VK_POLYGON_MODE_LINE);
		vk_str_case(type, VK_POLYGON_MODE_FILL);
		vk_str_case(type, VK_PRIMITIVE_TOPOLOGY_LINE_LIST);
		vk_str_case(type, VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST);
	}
};

struct AssetLoader{
	template <typename T, typename Y> using pair = std::pair<T, Y>;
	template <typename T> using vector = std::vector<T>;

	static vector<char> read_file(const std::string& filename){
		std::ifstream file(filename, std::ios::ate | std::ios::binary);

		if (!file.is_open()){
			throw std::runtime_error("failed to open file!");
		}

		size_t fileSize = (size_t)file.tellg();
		vector<char> buffer(fileSize);

		file.seekg(0);
		file.read(buffer.data(), fileSize);

		file.close();

		return buffer;
	}

	static json load_json_file(const std::string& filepath){
		std::ifstream t(filepath);
		std::string buf((std::istreambuf_iterator<char>(t)),
			std::istreambuf_iterator<char>());
		json j = json::parse(buf);
		t.close();
		return j;
	}

	static std::string build_asset_path(const json& asset_path,
		const json& relpath){
		return (std::string)asset_path + "/" + (std::string)relpath;
	}

	static std::map<std::string, ShaderSpirV> load_shaders(const json& asset_path, const json& shaders){
		std::map<std::string, ShaderSpirV> shaderMap;
		for (const auto& [key, shader] : shaders.items()){
			std::string vertexPath = build_asset_path(asset_path, shader["vert"]);
			std::string fragmentPath = build_asset_path(asset_path, shader["frag"]);
			int64_t polygonMode = VK_POLYGON_MODE_FILL, topologyType = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;

			if (shader.find("polygon_mode") != shader.end())
				polygonMode = ShaderSpirV::getTypeFromString(shader["polygon_mode"]);

			if (shader.find("topology_type") != shader.end())
				topologyType = ShaderSpirV::getTypeFromString(shader["topology_type"]);

			shaderMap[key] = {
				read_file(vertexPath + "_vert.spv"),
				read_file(fragmentPath + "_frag.spv"),
				polygonMode, topologyType};
		}
		return shaderMap;
	}

	//   load_images(const json &textureMap,
	//               std::map<std::string, vulkan::Image> &imageMap) {
	//     for (const auto &[key, value] : settings["textures"].items()) {
	//       imageMap[key] =
	//           vulkan::Image(ctx.device, ctx.physicalDevice,
	//           ctx.commandPool.handle,
	//                         buildAssetPath(value));
	//       imageMap[key].createTextureView(ctx.device);
	//       imageMap[key].createSampler(ctx.device, ctx.physicalDevice);
	//     }
	//   }
};
} // namespace xn