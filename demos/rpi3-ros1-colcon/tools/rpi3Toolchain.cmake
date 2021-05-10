
# Example toolchain for raspberry Pi 64 bit

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_VERSION 1)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc-10)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++-10)

if(DEFINED ENV{MY_SYSROOT})
    MESSAGE(STATUS "Sysroot at: $ENV{MY_SYSROOT}")
else()
    message(FATAL_ERROR "MY_SYSROOT environment variable must be defined, use 'export MY_SYSROOT=/path/to/my/sysroot'")
endif()

set(CMAKE_SYSROOT $ENV{MY_SYSROOT})
set(CMAKE_FIND_ROOT_PATH ${CMAKE_SYSROOT})

# Pkg config tweaks:
SET(ENV{PKG_CONFIG_LIBDIR} ${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu/pkgconfig)
SET(ENV{PKG_CONFIG_SYSROOT_DIR} ${CMAKE_SYSROOT})

set(PYTHON_SOABI cpython-38-aarch64-linux-gnu)
