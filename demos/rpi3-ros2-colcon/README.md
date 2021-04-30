
Demo of how to build ROS2 packages with the sysroot-creator and colcon.

Notable features:

- Use colcon as build tool
- Pass colcon a CMAKE_TOOLCHAIN_FILE
- Use AppImage to create an executable ROS2 app

# Preparation

Run these commands to prepare your environment:

    $ sudo apt install gcc-aarch64
    $ cd tools
    $ bash download-app-image-tools.sh
    $ sysroot-creator sysroot-rpi3.toml
    $ cd ..

# Build

The steps to build are outlined in this script:

    $ bash cross-build.sh

If all is well, you should have an AppImage at your hands, which can
be run on the raspberry pi.
