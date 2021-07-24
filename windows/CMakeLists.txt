cmake_minimum_required(VERSION 3.3)
set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Zi") 

include(C:/Code/vcpkg/scripts/buildsystems/vcpkg.cmake)
set(PROJ_DIR  ${CMAKE_CURRENT_SOURCE_DIR}/../../)
set(BUILD_DIR ${CMAKE_CURRENT_SOURCE_DIR}/build/Release)

set(XNLIB_DIR ${PROJ_DIR}/xnlib)
set(XN_VK_DIR ${XNLIB_DIR}/graphics/vulkan)
set(IMGUI_DIR ${XNLIB_DIR}/external/imgui)
set(GLM_DIR ${XNLIB_DIR}/external/glm )
set(OPENVINO_DIR "C:/Program Files (x86)/Intel/openvino_2021/opencv/cmake")
# set(OPENVINO_DIR C:/Code/opengl-es/opencv/build/x64/vc15/lib)
set(GLFW_DIR ${PROJ_DIR}/glfw-3.3.3.bin.WIN64)
set(GLAD_DIR ${PROJ_DIR}/glad)

set(libssh_DIR C:/Code/vcpkg/packages/libssh_x64-windows/share/libssh)
set(OpenCV_DIR ${OPENVINO_DIR})

set(COMMON_INCLUDE_DIRS
    ${GLFW_DIR}/include
    ${XNLIB_DIR}  
    ${GLAD_DIR}/include 
    ${IMGUI_DIR} ${IMGUI_DIR}/backends
    ${GLM_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../
    ${CMAKE_CURRENT_SOURCE_DIR}/../arm/src
)

set(PRECOMPILED_HEADERS
    <winsock2.h> <ws2tcpip.h> <windows.h>
    <stdio.h> <stdlib.h> <io.h> 
    <conio.h> <errno.h> <string.h> 
    <sys/types.h> <ctime> <iostream> 
    <fstream> <streambuf> <sstream> 
    <chrono> <thread> <queue> 
    <mutex> <cmath> <unordered_map> 
    <vector> <algorithm> <cstring> 
    <type_traits> <utility> <filesystem>
    <glm/glm.hpp> <glm/gtc/matrix_transform.hpp> 
    <glm/gtc/type_ptr.hpp> <glm/gtx/norm.hpp> 
    <glm/gtx/rotate_vector.hpp> 
    <opencv2/opencv.hpp>
)

# build static libraries
# -----------------------------------------------------------------------------

project(opengl_impl)
include_directories(${COMMON_INCLUDE_DIRS})
add_library(opengl_impl STATIC 
    ${PROJ_DIR}/glad/src/glad.c
    src/opengl/stb_image_impl.cpp
)

project(imgui)
include_directories(${COMMON_INCLUDE_DIRS})
add_library(imgui STATIC 
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_draw.cpp 
    ${IMGUI_DIR}/imgui_demo.cpp 
    ${IMGUI_DIR}/imgui_tables.cpp 
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp 
)


project(robolib)
find_package(OpenCV REQUIRED )
find_package(libssh CONFIG REQUIRED)
include_directories(${COMMON_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${libssh_DIR}/../../include)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../arm ${CMAKE_CURRENT_SOURCE_DIR}/../arm/build)

add_library(robolib STATIC 
    ${XNLIB_DIR}/util/xn_json.cpp
    src/robo/xn_pointcloud.cpp
    src/robo/xn_search.cpp
    src/robo/xn_yolo.cpp
    src/robo/xn_ssh.cpp
    src/robo/xn_log.cpp
    src/robo/xn_net.cpp
    src/robo/xn_homo.cpp
    src/robo/xn_car_virtual.cpp
    src/robo/xn_console.cpp
    src/robo/xn_rover.cpp
    src/app/app_threads_shared.cpp
    src/app/app_state.cpp
)

# build macros
# -----------------------------------------------------------------------------
macro(GL_Build project_name src_file )
    project( ${project_name} ) 
    set(CMAKE_PREFIX_PATH ${OPENVINO_DIR})

    find_package( OpenCV REQUIRED )
    find_package(libssh CONFIG REQUIRED)

    add_executable(${PROJECT_NAME} 
        ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
        ${ARGN}
        ${src_file} 
    )

    target_link_options(${PROJECT_NAME} PUBLIC /DEBUG)
    target_compile_options(${PROJECT_NAME} PRIVATE /Bt)
    add_definitions(-D_USE_MATH_DEFINES)

    target_include_directories( ${PROJECT_NAME} PRIVATE 
        ${COMMON_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
    )

    link_directories( ${GLFW_DIR}/lib-vc2019)
    target_link_libraries( ${PROJECT_NAME} 
        ${GLFW_DIR}/lib-vc2019/glfw3.lib 
        xnlib opengl_impl imgui 
        Ws2_32.lib ${OpenCV_LIBS} ssh
    )

    target_precompile_headers(${PROJECT_NAME} PRIVATE 
        ${PRECOMPILED_HEADERS}
        <glad/glad.h> <GLFW/glfw3.h> <libssh/libssh.h>
        ${IMGUI_DIR}/imgui.h ${IMGUI_DIR}/backends/imgui_impl_glfw.h 
        ${IMGUI_DIR}/backends/imgui_impl_opengl3.h 
    )

    # copy opencv dlls
    foreach(dll ${ocv_dll})
        message(${dll})
        file(COPY ${dll} DESTINATION ${BUILD_DIR})
    endforeach(dll)
endmacro()

macro(VK_Build project_name src_file)
    project( ${project_name} ) 
    set (extra_macro_args ${ARGN})

    find_package(Vulkan REQUIRED)
    find_package(OpenCV REQUIRED )
    find_package(libssh CONFIG REQUIRED)
    # find_library(SDL2_LIBRARY NAME SDL2 PATHS ${SDL_DIR}/lib/x64)
    # find_file(SDL2_INCLUDE_DIR NAME SDL.h PATHS ${SDL_DIR}/include HINTS SDL2 )

    add_subdirectory(${XN_VK_DIR} ${XN_VK_DIR}/build)

    add_executable( ${PROJECT_NAME}  
        ${XNLIB_DIR}/external/vk_imgui/imgui_impl_vulkan.cpp
        ${XNLIB_DIR}/external/stb_image_impl.cpp
        ${ARGN}
        ${src_file} 
    )

    target_compile_options(${PROJECT_NAME} PRIVATE /Bt)

    target_link_directories(${PROJECT_NAME} PRIVATE ${GLFW_DIR}/lib-vc2019 ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS} 
        armlib 
        robolib 
        Vulkan::Vulkan xn_vulkan imgui
        ${GLFW_DIR}/lib-vc2019/glfw3.lib ssh Ws2_32.lib       
    )

    target_include_directories(${PROJECT_NAME} PRIVATE
        ${COMMON_INCLUDE_DIRS}
        ${Vulkan_INCLUDE_DIR}
    )

    target_precompile_headers(${PROJECT_NAME} PRIVATE ${PRECOMPILED_HEADERS})
endmacro()


# build the darn apps
# -----------------------------------------------------------------------------

# GL_Build( lidar_gl src/lidar_gl.cpp ${src_obj_list})
VK_Build( lidar_vk src/lidar_vk.cpp )