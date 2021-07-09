# Compile with opengl, glad, and glfw3 for WSL(software renderer at time of writing)

cmake_minimum_required(VERSION 3.11)
set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_CXX_FLAGS " ${CMAKE_CXX_FLAGS} -Wall -g")
set(CMAKE_C_FLAGS " ${CMAKE_C_FLAGS} -Wall -g")

project(opengl_impl)
set( opengl_impl_src  
    ./glad/src/glad.c
    stb_image_impl.cpp
)
include_directories(./glad/include )
add_library(opengl_impl STATIC ${opengl_impl_src} )

set(IMGUI_DIR /home/abobco/RaspberryPi/imgui)
project(imgui)
set(imgui_src 
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_draw.cpp 
    ${IMGUI_DIR}/imgui_tables.cpp 
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp 
    ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
)
include_directories(${IMGUI_DIR} ${IMGUI_DIR}/backends)
add_library(imgui STATIC ${imgui_src} )


macro(GL_Build project_name src_file )
    project( ${project_name} ) 

    find_package(Freetype REQUIRED)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_FLAGS " ${CMAKE_CXX_FLAGS} -Wall  -std=c++17 -g")
    set(CMAKE_C_FLAGS " ${CMAKE_C_FLAGS} -Wall -g")

    set (extra_macro_args ${ARGN})
    # Did we get any optional args?
    list(LENGTH extra_macro_args num_extra_args)
    if (${num_extra_args} GREATER 0)
        # compile with profiler
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
        set(CMAKE_C_FLAGS " ${CMAKE_C_FLAGS} -O3") 
    endif()

    set(CMAKE_BUILD_TYPE Debug)

    add_executable(${PROJECT_NAME}  ${src_file} )
    target_link_libraries(${PROJECT_NAME} ${FREETYPE_LIBRARIES} opengl_impl glfw GL X11 pthread Xrandr Xi dl Xxf86vm stdc++fs imgui)
    target_include_directories(${PROJECT_NAME} PRIVATE ${FREETYPE_INCLUDE_DIRS} ${IMGUI_DIR} ${IMGUI_DIR}/backends)
endmacro()

GL_Build( lidar lidar_gl.cpp)