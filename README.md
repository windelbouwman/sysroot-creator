
# Introduction

This sysroot creator can create a sysroot for use in cross compilation.
The basic idea is to download package files for the intended target
and extract those packages in a folder.

For example, when targetting a raspberry Pi with ubuntu the process
consists of:

- Download the ubuntu package databases for arm64
- Select the packages to download
- Download and extract those deb packages to a folder

Notable issues which are handled during this process:
- Some packages contain symlinks to absolute paths. These are modified
  to a new absolute path, so that the symlink is valid.
- Some packages contain special files, such as `/dev/null`. These files
  are skipped, since they are not required for cross compilation. Only
  header files, libraries and other regular files are unpacked.

# Usage

Create a configuration file, for example `my-sysroot-conf.toml`, describing your intended sysroot:

```toml
    distribution_version = "focal"
    architecture = "arm64"
    folder = "sysroot-rpi3"

    packages = [
        "libcrypt-dev",
        "libtinyxml2-dev",
        "ros-foxy-rclcpp",
    ]

    [sources]

    [sources.ports]
    url = "http://ports.ubuntu.com/ubuntu-ports"
    sections = [
        "main",
        "universe",
    ]

    [sources.ros2]
    url = "http://packages.ros.org/ros2/ubuntu"
    sections = [
        "main",
    ]
```

Run this program to create the sysroot:

    $ cargo run --release -- my-sysroot-conf.toml

Your sysroot will be located in the folder `sysroot-rpi3/sysroot`.

Now you can use this sysroot for cross compilation, using for example
a CMake toolchain file.

# How does it work?

The sysroot creator is a rust application. It first parses the toml
file with the settings.

Given these settings, it downloads package database from the given
sources. Those files are cached. The contents of the package databases
are parsed.

Next up is package selection. Based on the intended packages, dependencies
are resolved, and a list of desired packages is created.

Each package is downloaded and cached. Then it is unpacked, and
it's files are placed in the destination folder. Special files
(block / character device files) are skipped, and absolute symlinks are
handled.

# Alternatives

There are many different ways to setup a sysroot for cross compilation.

- Use an actual target, and scp the root filesystem onto your development PC.
  This could be a fairly simple strategy. Take care to fix eventual symlinks
  when doing this.
- Use `debootstrap` to do a cross debootstrap. I was not yet able to do this.
  This seems worth a try.
- Use [labapart/cross_sysroot](https://github.com/labapart/cross_sysroot). This project
  gave me the idea to attempt this version of the idea in rust. Features I added
  was the idea of a configuration file, and support for multiple repositories.
  Also, this project does not use dpkg to extract packages.
