cmake_minimum_required(VERSION 3.11) # FetchContent is available in 3.11+
include(FetchContent)
include(armhf-toolchain.cmake)
set(CMAKE_BUILD_TYPE Debug)
# raspi libraries for threading & gpu
# include_directories( ${CMAKE_SYSROOT}/opt/vc/include ${CMAKE_SYSROOT}/opt/vc/include/interface/vmcs_host/linux ${CMAKE_SYSROOT}/opt/vc/include/interface/vcos/pthreads ${CMAKE_SYSROOT}/opt/vc/include/interface/vcos/generic)
# link_directories(${CMAKE_SYSROOT}/opt/vc/lib)

# ========================== Build libwebsockets ========================================

project(lws)

# LWS config
set(OPENSSL_ROOT_DIR ${CMAKE_SYSROOT}/usr/lib/arm-linux-gnueabihf/)
set(CMAKE_BUILD_WITH_INSTALL_RPATH ON)
set(CMAKE_TOOLCHAIN_FILE armhf-toolchain.cmake)
set(LWS_WITH_TLS ON)

 # build a debug symbol table for gdb
set(CMAKE_BUILD_TYPE Debug)

# fetch & build libwebsockets
FetchContent_Declare(
    libwebsockets
    GIT_REPOSITORY https://github.com/warmcat/libwebsockets.git
    GIT_TAG v4.0.10
    )
  FetchContent_GetProperties(libwebsockets)
  if(NOT libwebsockets_POPULATED)
    set(FETCHCONTENT_QUIET NO)
    FetchContent_Populate(libwebsockets)
    add_subdirectory(${libwebsockets_SOURCE_DIR} ${libwebsockets_BINARY_DIR})
  endif()
  
  # libwebsockets build dependencies
  include_directories(build/_deps/libwebsockets-build/include ${CMAKE_SYSROOT}/usr/include/arm-linux-gnueabihf)
  link_directories(build/_deps/libwebsockets-build/lib ${CMAKE_SYSROOT}/usr/lib/arm-linux-gnueabihf/openssl-1.0.2)

# =======================================================================================

# ================================ Helper Library ========================================

project(robolib_build)

set( arducam_dir /home/abobco/RaspberryPi/OCV_Projects/SPI_Camera/src/ )

set( robolib_src  
    src/ws/message_queue.c 
    ${arducam_dir}EZCAM.c 
    ${arducam_dir}ArduCAM.c 
    ${arducam_dir}bcm283x_board_driver.c 
    ${arducam_dir}sccb_bus.c
)

# link_directories(build/_deps/raylib-build/src)

add_library(robolib STATIC ${robolib_src} )
# target_link_libraries( ${PROJECT_NAME} rt )

# ========================================================================================

macro(Robo_Build project_name src_file)
    project( ${project_name} ) 
    
    # build a debug symbol table for gdb
    set(CMAKE_BUILD_TYPE Debug)

    add_executable(${PROJECT_NAME}  ${src_file} )

    find_package(OpenCV REQUIRED)
    include_directories(SYSTEM /home/abobco/RaspberryPi/pigpio-master/install/include )
    include_directories(${CMAKE_SYSROOT}/opt/vc/include ${CMAKE_SYSROOT}/opt/vc/include/interface/vmcs_host/linux)
    include_directories(/home/abobco/RaspberryPi/OCV_Projects/SPI_Camera/src )
    include_directories(/home/abobco/RaspberryPi/OCV_Projects/raspicam/src )
    link_directories( /home/abobco/RaspberryPi/pigpio-master/install/lib/)
    find_library(PIG NAMES pigpio PATHS /home/abobco/RaspberryPi/pigpio-master/install/lib/ NO_DEFAULT_PATH )
    find_library(PIGD NAMES pigpiod_if2 PATHS /home/abobco/RaspberryPi/pigpio-master/install/lib/ NO_DEFAULT_PATH )

    link_directories(${CMAKE_SYSROOT}/opt/vc/lib/ /home/abobco/RaspberryPi/OCV_Projects/raspicam/build/src)

    find_library(l_EGL NAMES EGL PATHS ${CMAKE_SYSROOT}/opt/vc/lib/)
    find_library(l_GLES NAMES brcmGLESv2 PATHS ${CMAKE_SYSROOT}/opt/vc/lib/)
    find_library(l_bcm_host NAMES bcm_host PATHS ${CMAKE_SYSROOT}/opt/vc/lib/)
    find_library(l_vcos NAMES vcos PATHS ${CMAKE_SYSROOT}/opt/vc/lib/)
    find_library(l_vchiq_arm NAMES vchiq_arm PATHS ${CMAKE_SYSROOT}/opt/vc/lib/)
    find_library(l_mmal_core NAMES mmal_core PATHS ${CMAKE_SYSROOT}/opt/vc/lib/)
    find_library(l_mmal NAMES mmal PATHS ${CMAKE_SYSROOT}/opt/vc/lib/)
    find_library(l_mmal_util NAMES mmal_util PATHS ${CMAKE_SYSROOT}/opt/vc/lib/)
    # set(raspicam_DIR /home/abobco/RaspberryPi/OCV_Projects/raspicam/build/)
    # find_package(raspicam REQUIRED )

    # set(raspicam_LIBS 
    #       /home/abobco/RaspberryPi/OCV_Projects/raspicam/build/src/libraspicam.a
    #       )

    # target_link_libraries( ${PROJECT_NAME}  ${OpenCV_LIBS} ${PIG} ${PIGD} pthread rt robolib websockets dl ${l_vcos} ${l_EGL} ${l_GLES} ${l_bcm_host} ${l_vchiq_arm})
    target_link_libraries( ${PROJECT_NAME} 
                           ${OpenCV_LIBS}
                           ${PIG} ${PIGD} 
                           pthread rt 
                           robolib 
                           websockets dl 
                           ${l_vcos}  ${l_GLES} ${l_bcm_host} ${l_vchiq_arm} 
                           ${l_mmal_core} ${l_mmal} ${l_mmal_util}
                           /home/abobco/RaspberryPi/OCV_Projects/raspicam/build/src/libraspicam.a  
                           /home/abobco/RaspberryPi/OCV_Projects/raspicam/build/src/libraspicam_cv.a  
    )

    # cross compiler stuff
    set_target_properties(${PROJECT_NAME} PROPERTIES BUILD_WITH_INSTALL_RPATH TRUE)
    set_target_properties(${PROJECT_NAME} PROPERTIES INSTALL_RPATH "./")
endmacro()

# simulation build
# Robo_Build( egl_info src/XNlib/egl/egl_info.cpp )
# Robo_Build( egl_test src/XNlib/egl/egl_tests.cpp )
# Robo_Build( egl_tri src/XNlib/opengl/egl_tri.cpp )

# deprecated programs (use original pigpio interface instead of pigpiod)
# Robo_Build( servo_smooth src/servo_smooth.cpp )
# Robo_Build( servo_smooth_array src/servo_smooth_array.cpp )
# Robo_Build( arm_lws src/arm_lws.cpp )
# Robo_Build( servo360 src/servo360.cpp )
# Robo_Build( xn_test_phys src/XNlib/xn_test_phys.cpp )
# Robo_Build( tfmini src/tfmini.cpp )
# Robo_Build( lidar_net src/lidar_net.cpp )

Robo_Build( arm_controller_main src/arm_controller_main.cpp )
Robo_Build( servo src/servo_demo.cpp )
Robo_Build( sonar src/sonar.cpp )
Robo_Build( pano src/pano_controller.cpp )
Robo_Build( stepper src/stepper.cpp )
Robo_Build( red src/rot_enc.cpp )
Robo_Build( wavetimer src/wavetimer.cpp )
Robo_Build( i2c_ctl src/i2c_controller.cpp )
Robo_Build( sock_client src/sock_client.cpp )
Robo_Build( arducam_sock src/arducam_sock.cpp)
Robo_Build( arducam_minimal src/arducam_minimal.cpp)
Robo_Build( serial_read src/serial_read.cpp)
Robo_Build( picam_minimal src/picam_minimal.cpp)
Robo_Build( picam_stream src/picam_stream.cpp)
Robo_Build( picam_cv_minimal src/picam_cv_minimal.cpp)
Robo_Build( picam_cv_stream src/picam_cv_stream.cpp)
Robo_Build( picam_cv_stream_lossless src/picam_cv_stream_lossless.cpp)
Robo_Build( led_pwm src/led_pwm.cpp)