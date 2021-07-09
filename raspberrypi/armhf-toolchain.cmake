# arm-hardfloat toolchain file for a Linux host using raspberrypi-tools(https://github.com/raspberrypi/tools)

set(CMAKE_CROSSCOMPILING TRUE)
SET(CMAKE_SYSTEM_NAME Linux)
SET(CMAKE_SYSTEM_PROCESSOR arm)



# give path to prebuilt toolchain
# set(TOOLCHAIN_PATH "D:/SysGCC" CACHE PATH "Path to your raspberry toolchain")
# set(GCC_ARM_TOOLCHAIN "${TOOLCHAIN_PATH}/raspberry/arm-linux-gnueabihf")
# set(GCC_ARM_LINKER_EXE "D:/SysGCC/raspberry/bin/arm-linux-gnueabihf-ld.exe" CACHE PATH "Linker for the target system")
SET(TARGET_TRIPLE arm-linux-gnueabihf)

SET(CMAKE_SYSROOT /home/abobco/RaspberryPi/sysroot)


# get the toolchain sysroot
# LIST(APPEND CMAKE_PROGRAM_PATH ${GCC_ARM_TOOLCHAIN})
# SET(CMAKE_SYSROOT ${GCC_ARM_TOOLCHAIN}/sysroot)

# set compilers
SET(CMAKE_C_COMPILER  /home/abobco/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin/arm-linux-gnueabihf-gcc)
SET(CMAKE_CXX_COMPILER /home/abobco/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin/arm-linux-gnueabihf-g++)
# SET(CMAKE_C_COMPILER_TARGET ${TARGET_TRIPLE})
# SET(CMAKE_CXX_COMPILER_TARGET ${TARGET_TRIPLE})
# set(CMAKE_ASM_COMPILER_TARGET ${TARGET_TRIPLE})
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# linker flags
# set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fuse-ld=${GCC_ARM_LINKER_EXE} -g")
# set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fuse-ld=${GCC_ARM_LINKER_EXE} -g")

# compiler flags
set(CMAKE_CXX_FLAGS " ${CMAKE_CXX_FLAGS} -Wall -g -std=gnu++11 -O3")
set(CMAKE_C_FLAGS " ${CMAKE_C_FLAGS} -Wall -g -std=gnu99 -O3")

# Search for programs in the build host directories
set(CMAKE_FIND_ROOT_PATH ${CMAKE_SYSROOT})
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
# For libraries and headers in the target directories
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY BOTH)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)