#!/bin/bash

# Helper script to wget the appropriate app image utilities.

set -eu

THIS_FOLDER=$(dirname $(realpath $BASH_SOURCE))
cd ${THIS_FOLDER}

if [ -f appimagetool-x86_64.AppImage ]; then
    echo "appimagetool-x86_64.AppImage is present!"
else
    echo "Downloading appimagetool-x86_64.AppImage"
    wget https://github.com/AppImage/AppImageKit/releases/download/12/appimagetool-x86_64.AppImage
    chmod +x appimagetool-x86_64.AppImage
fi

if [ -f runtime-aarch64 ]; then
    echo "runtime-aarch64 is present!"
else
    echo "Downloading runtime-aarch64"
    wget https://github.com/AppImage/AppImageKit/releases/download/12/runtime-aarch64
fi
