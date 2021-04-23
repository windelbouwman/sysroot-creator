

Demo on how to cross compile a ROS2 app for a raspberry Pi 3.

# Usage

Ensure you have a ros2 environment:

    $ source /opt/ros/foxy/local_setup.bash

Create a sysroot:

    $ sysroot-creator sysroot-rpi3.toml

Patch a single cmake file (in future, this should not be required):

    $ bash patch-ros2-stuff.sh

Build the example app:

    $ ./cross-build.sh

You can ship the install folder to your target, and use it there.
Note that you must setup the variables `LD_LIBRARY_PATH` and `PYTHONPATH`
properly for this to work out.

