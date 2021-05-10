#!/bin/bash

set -eu

ROS1_FOLDER=./sysroot-rpi3/sysroot/opt/ros/noetic

if [ ! -d ${ROS1_FOLDER} ]; then
    echo "Could not find ROS folder: ${ROS1_FOLDER}"
    exit 1
fi

# Issue:
# this is problematic: "roscpp;pthread;/usr/lib/aarch64-linux-gnu/libboost_chrono.so.1.71.0;/usr/lib/aarch64-linux-gnu/libboost_filesystem.so.1.71.0;/usr/lib/aarch64-linux-gnu/libboost_system.so.1.71.0"
# Idea: look into cmake files and modify lines like '/lib/aarch64/libboost.so.1.70.0' into 'boost'
# use 'find' to loop over all cmake files
# use 'sed' to find / replace this pattern
# sed concept line: s+/lib/aarch64/lib().so/+\1+g
# Explainer:
# - s = substitute
# - use + instead of / to 
# - g = globally (all occurences per line)

# Proof-of-concept sed line:
TEST_LINE="roscpp;pthread;/usr/lib/aarch64-linux-gnu/libpython3.8.so;/usr/lib/aarch64-linux-gnu/libboost_chrono.so.1.71.0;/usr/lib/aarch64-linux-gnu/libboost_filesystem.so.1.71.0;/usr/lib/aarch64-linux-gnu/libboost_system.so.1.71.0"
echo ${TEST_LINE}
echo ${TEST_LINE} \
 | sed 's@/usr/lib/aarch64-linux-gnu/lib\([A-Za-z_0-9.]\+\).so[0-9.]*@\1@g'

# List all cmake files:
# find ${ROS1_FOLDER} -name "*.cmake"

# Grep all cmake files:
# find ${ROS1_FOLDER} -name "*.cmake" -exec grep "/usr/lib" {} \;


# Patch cmake files:
# TODO: make this work ..
patch_cmake_file () {
    find ${ROS1_FOLDER} -name "*Config.cmake" | while read CMAKE_FILE; do
        # echo "Testing: ${CMAKE_FILE}"
        BAK_CMAKE_FILE="${CMAKE_FILE}.bak"

        if [ ! -f ${BAK_CMAKE_FILE} ]; then
            echo "Creating backup: ${BAK_CMAKE_FILE}"
            cp ${CMAKE_FILE} "${BAK_CMAKE_FILE}"
        fi

        # Change absolute paths into library names only:
        # Turn lines like '/usr/lib/aarch64-linux-gnu/libboost.so.1.70.0' into 'boost'
        if grep --color --line-number "/usr/lib/aarch64-linux-gnu/lib" ${CMAKE_FILE}; then
            echo "Patching: ${CMAKE_FILE}"
            sed --in-place --expression 's@/usr/lib/aarch64-linux-gnu/lib\([A-Za-z_0-9.]\+\).so[0-9.]*@\1@g' ${CMAKE_FILE}
        fi

        # Remove NO_CMAKE_FIND_ROOT_PATH, since this prevents proper
        # usage of our sysroot folder.
        if grep --color --line-number "NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH" ${CMAKE_FILE}; then
            echo "Patching: ${CMAKE_FILE}"
            sed --in-place --expression 's@NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH@NO_DEFAULT_PATH@g' ${CMAKE_FILE}
        fi

        if [ -f ${BAK_CMAKE_FILE} ]; then
            echo "Changes to: ${CMAKE_FILE}"
            diff --unified --color "${BAK_CMAKE_FILE}" "${CMAKE_FILE}" || true
        fi

    done
}

# TODO: create loop to restore files

patch_cmake_file

# Fix permissions of templates
# This is a bit weird, but is required in order to build.
# The template is applied, and the generated file is executed during
# catkin build.
CATKIN_CMAKE_TEMPLATES_FOLDER=${ROS1_FOLDER}/share/catkin/cmake/templates
ls -l ${CATKIN_CMAKE_TEMPLATES_FOLDER}
chmod +x ${CATKIN_CMAKE_TEMPLATES_FOLDER}/env.sh.in
chmod +x ${CATKIN_CMAKE_TEMPLATES_FOLDER}/_setup_util.py.in
ls -l ${CATKIN_CMAKE_TEMPLATES_FOLDER}
