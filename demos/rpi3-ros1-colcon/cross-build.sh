#!/bin/bash

set -eu

THIS_FOLDER=$(dirname $(realpath $BASH_SOURCE))
echo "Assuming located here: ${THIS_FOLDER}"

cd ${THIS_FOLDER}

COLCON_WS=cross_ws

# Create a workspace, or keep existing one:
if [ -d ${COLCON_WS} ]; then
    echo "Using existing workspace: ${COLCON_WS}"
else
    echo "Creating workspace: ${COLCON_WS}"
    mkdir ${COLCON_WS}
    ln -s ${THIS_FOLDER}/src ${COLCON_WS}/src
fi

cd ${COLCON_WS}

# Use this sysroot, picked up by toolchain file:
export MY_SYSROOT=${THIS_FOLDER}/tools/sysroot-rpi3/sysroot

# Select toolchain file:
CMAKE_TOOLCHAIN_FILE=${THIS_FOLDER}/tools/rpi3Toolchain.cmake

# for more output, use VERBOSE=1 
colcon build --merge-install \
    --event-handlers console_direct+ --parallel-workers 1 \
    --cmake-args -DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}

#MY_ROS_FOLDER=${MY_SYSROOT}/opt/ros/noetic

#export ROS_PACKAGE_PATH=${MY_ROS_FOLDER}/share
#export ROS_ROOT=${MY_ROS_FOLDER}/share/ros
#export ROS_ETC_DIR=${MY_ROS_FOLDER}/etc/ros

# -DCMAKE_PREFIX_PATH=${MY_SYSROOT}/opt/ros/noetic
# catkin_make_isolated -DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}

