#include "App.h"

using namespace xn;

static const std::string DEFAULT_SETTINGS_FILE_PATH =
"C:/Code/opengl-es/robot-host-app/windows/assets/config/vk_config.json";
std::string settings_filepath = DEFAULT_SETTINGS_FILE_PATH;

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