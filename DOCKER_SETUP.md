# ZED ROS Wrapper Docker Setup

This document outlines the requirements and steps to containerize the ZED ROS wrapper using Docker.

## 1. System Requirements

- **ROS Version**: ROS Noetic Ninjemis
- **Operating System**: Ubuntu 20.04 (Focal Fossa)
- **ZED SDK Version**: 4.1
- **NVIDIA Driver**: A compatible NVIDIA driver must be installed on the host machine.
- **NVIDIA Container Runtime**: The [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) must be installed on the host machine to enable GPU access within the Docker container at runtime.

## 2. Docker Image Components

The Docker image will be built upon the following components:

- **Base Image**: `ros:noetic-ros-base-focal`
- **System Dependencies**: `build-essential`, `wget`, `cmake`, `libopencv-dev`, `python3-pip`, `zstd`, `sudo`, `less`, `file`, `libblas-dev`, `liblapack-dev`, and `libusb-1.0-0-dev`.
- **NVIDIA CUDA Toolkit**: Version 12.1 will be installed in the image to allow the ZED SDK to be built correctly.
- **ZED SDK**: Version 4.1 for Ubuntu 20.04. The SDK will be downloaded and installed within the Docker image.
- **ROS Dependencies**: All necessary ROS packages required by the `zed-ros-wrapper` will be installed, including `robot_state_publisher`, `image_transport_plugins`, and `xacro`.
- **Project Code**: The project source code will be copied into the image and built using `catkin_make`.

## 3. Dockerfile

The `Dockerfile` performs the following steps:

1.  Starts from the `ros:noetic-ros-base-focal` base image.
2.  Installs system dependencies, including `sudo`, `less`, `file`, `libblas-dev`, `liblapack-dev`, `libusb-1.0-0-dev`, and the `zstd` decompression tool.
3.  Installs additional ROS packages: `ros-noetic-robot-state-publisher`, `ros-noetic-image-transport-plugins`, and `ros-noetic-xacro`.
4.  Downloads and installs the NVIDIA CUDA Toolkit v12.1. This is a critical step for the ZED SDK installation.
5.  Creates a temporary `builder` user with `sudo` privileges, as the ZED SDK installer requires being run by a non-root user.
6.  Gives the `builder` user ownership of the `/usr/local/zed` directory.
7.  Switches to the `builder` user to download and run the ZED SDK installer.
8.  Switches back to the `root` user.
9.  Creates a catkin workspace.
10. Copies the `zed-ros-wrapper` source code into the workspace.
11. Installs the ROS dependencies for the project.
12. Builds the catkin workspace, providing the path to the ZED SDK's CMake files.
13. Sets up an entrypoint to source the ROS environment.

## 4. Usage

To build the Docker image:

```bash
docker build -t zed-ros-wrapper .
```

To run the Docker container:

```bash
docker run -d --gpus all --privileged --network=host --ipc=host -v /dev:/dev zed-ros-wrapper
```