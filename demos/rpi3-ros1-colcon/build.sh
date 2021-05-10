#!/bin/bash

set -eu

THIS_FOLDER=$(dirname $(realpath $BASH_SOURCE))
echo "Assuming located here: ${THIS_FOLDER}"

cd ${THIS_FOLDER}

COLCON_WS=ws

if [ ! -d ${COLCON_WS} ]; then
    echo "Creating workspace: ${COLCON_WS}"
    mkdir ${COLCON_WS}
    ln -s ${THIS_FOLDER}/src ${COLCON_WS}/src
fi

cd ${COLCON_WS}

colcon build --merge-install \
    --event-handlers console_direct+ --parallel-workers 1
