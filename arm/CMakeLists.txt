project(armlib)
include_directories(${COMMON_INCLUDE_DIRS})
add_library(armlib STATIC 
    src/xn_ik.cpp
    src/xn_vec.cpp
    src/xn_math.cpp
    src/xn_arm.cpp
    src/xn_gpio.cpp
    src/xn_gpio_win32.cpp
)

target_include_directories(armlib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)