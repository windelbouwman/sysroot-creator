#!/bin/bash

# Source foxy environment:
if [ -v ROS_DISTRO ]; then
    echo "Using host ROS distro: ${ROS_DISTRO}"
else
    echo "Sourcing foxy environment first: /opt/ros/foxy/local_setup.bash"
    source /opt/ros/foxy/local_setup.bash
fi

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

colcon build --merge-install \
    --cmake-args -DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}

# Zip the install folder for usage on target:
DO_ZIP="no"
if [ ${DO_ZIP} == "yes" ]; then
    zip -r bundle.zip install
fi

# Create app image file (seperate script?):
APPIMAGETOOL=${THIS_FOLDER}/tools/appimagetool-x86_64.AppImage
APPIMAGERUNTIME=${THIS_FOLDER}/tools/runtime-aarch64

if [ -f ${APPIMAGETOOL} ]; then
    cd ${THIS_FOLDER}

    # Create some AppDir:
    APPDIR_FOLDER=AppDir
    if [ ! -d ${APPDIR_FOLDER} ]; then
        mkdir ${APPDIR_FOLDER}
    fi

    # Add some helper scripts:
    cp tools/appimage/{myapp.desktop,ros_icon.png,AppRun} ${APPDIR_FOLDER}/

    # Insert colcon install folder:
    cp -r ${COLCON_WS}/install ${APPDIR_FOLDER}/

    # Create portable app:
    ${APPIMAGETOOL} --runtime-file ${APPIMAGERUNTIME} ${APPDIR_FOLDER}
else
    echo "Not found: ${APPIMAGETOOL}, use ./download-app-image-tools.sh"
fi

# Deploy:
APPIMG=MyApp-aarch64.AppImage
# TODO: how to automate?
# echo "Deploying to target!"
# scp bundle.zip pi:/home/ubuntu/
# scp ${APPIMG} pi:/home/ubuntu/
