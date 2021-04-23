#!/bin/bash

set -eu

ROS2_ROOT=./sysroot-rpi3/sysroot/opt/ros/foxy

if [ ! -d ${ROS2_ROOT} ]; then
    echo "Cannot find folder: ${ROS2_ROOT}!"
    exit 1
fi

# Patch to work around absolute path issue:
FASTRTPT_TARGETS_CMAKE=${ROS2_ROOT}/share/fastrtps/cmake/fastrtps-targets.cmake
echo "Patching: ${FASTRTPT_TARGETS_CMAKE}"
sed -i -e s+/usr/lib/aarch64-linux-gnu/libtinyxml2.so+tinyxml2+ ${FASTRTPT_TARGETS_CMAKE}
# See also: https://github.com/eProsima/Fast-DDS/issues/1264
