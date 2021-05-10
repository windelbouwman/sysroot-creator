
**WARNING: DOES NOT WORK!!!**

This demo is ROS1 cross compiled to raspberry Pi 3.

# Preparation

Create a sysroot, and apply some patching to make things work out.

    $ cd tools
    $ sysroot-creator sysroot-rpi3.toml
    $ bash ../../../patch-ros1-stuff.sh

# Building

Build using colcon, by using this script:

    $ bash cross-build.sh

The resulting folder will be in `cross_ws/install`.

## Issues encountered

Interesting issue encountered:

Some template files here must be executable:
`/opt/ros/noetic/share/catkin/cmake/templates/`

- `env.sh.in`
- `_setup_util.py.in`

Some paths are absolute in cmake files.

The usage of `NO_CMAKE_FIND_ROOT_PATH` is causing issues,
best to remove it.

