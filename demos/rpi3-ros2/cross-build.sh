#!/bin/bash

set -eu

# The app to build:
APPNAME=app

THIS_FOLDER=$(dirname $(realpath $BASH_SOURCE))
echo "Assuming located here: ${THIS_FOLDER}"

cd ${THIS_FOLDER}

SRC_FOLDER=${APPNAME}
BUILD_FOLDER=${APPNAME}_cross_build
INSTALL_FOLDER=${APPNAME}_cross_install

export MY_SYSROOT=${THIS_FOLDER}/sysroot-rpi3/sysroot
MY_ROS_FOLDER=${MY_SYSROOT}/opt/ros/foxy
export CMAKE_PREFIX_PATH=${MY_ROS_FOLDER}
export AMENT_PREFIX_PATH=${MY_ROS_FOLDER}
TOOLCHAIN_FILE=${THIS_FOLDER}/rpi3Toolchain.cmake

if [ -d "$BUILD_FOLDER" ] && [ ! -f "$BUILD_FOLDER/build.ninja" ]; then
  echo "$BUILD_FOLDER/build.ninja not found, wiping build folder."
  rm -rf $BUILD_FOLDER
fi

if [ -d "$BUILD_FOLDER" ]; then
  echo "Folder $BUILD_FOLDER does exist!"
else
  echo "Folder $BUILD_FOLDER does not exist, creating it using cmake!"
  cmake -B $BUILD_FOLDER -G Ninja $SRC_FOLDER -DCMAKE_TOOLCHAIN_FILE=$TOOLCHAIN_FILE -DCMAKE_INSTALL_PREFIX=$INSTALL_FOLDER
fi

ninja -C $BUILD_FOLDER install

zip -r ${APPNAME}.zip ${INSTALL_FOLDER}
