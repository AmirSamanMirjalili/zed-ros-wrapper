# ZED ROS Wrapper Docker Setup

This document outlines the requirements and steps to containerize the ZED ROS wrapper using Docker.

## 1. System Requirements

- **ROS Version**: ROS Noetic Ninjemis
- **Operating System**: Ubuntu 20.04 (Focal Fossa)
- **ZED SDK Version**: 4.1
- **NVIDIA Driver**: A compatible NVIDIA driver must be installed on the host machine. If you encounter issues with GPU access, ensure your NVIDIA drivers are installed and working correctly. You can typically install them using `sudo apt install nvidia-utils-550` (or a similar version recommended for your system). After installation, run `nvidia-smi`. If it reports "NVIDIA-SMI has failed because it couldn't communicate with the NVIDIA driver", **you must reboot your computer**. Verify successful installation and communication with `nvidia-smi` after rebooting.
- **NVIDIA Container Runtime**: The [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) must be installed on the host machine to enable GPU access within the Docker container at runtime. If Docker reports "unknown or invalid runtime name: nvidia", you may need to configure `/etc/docker/daemon.json` as described in the "Docker Configuration for NVIDIA Runtime" section below.

## 2. Docker Image Components

The Docker image will be built upon the following components:

- **Base Image**: `nvidia/cuda:12.1.0-devel-ubuntu20.04`
- **System Dependencies**: `build-essential`, `wget`, `cmake`, `libopencv-dev`, `python3-pip`, `zstd`, `sudo`, `less`, `file`, `libblas-dev`, `liblapack-dev`, and `libusb-1.0-0-dev`.
- **ZED SDK**: Version 4.1 for Ubuntu 20.04. The SDK will be downloaded and installed within the Docker image.
- **ROS Dependencies**: All necessary ROS packages required by the `zed-ros-wrapper` will be installed, including `robot_state_publisher`, `image_transport_plugins`, and `xacro`.
- **Project Code**: The project source code will be copied into the image and built using `catkin_make`.

## 3. Dockerfile

The `Dockerfile` performs the following steps:

1.  Starts from the `nvidia/cuda:12.1.0-devel-ubuntu20.04` base image.
2.  Installs system dependencies, including `sudo`, `less`, `file`, `libblas-dev`, `liblapack-dev`, `libusb-1.0-0-dev`, and the `zstd` decompression tool.
3.  Installs additional ROS packages: `ros-noetic-robot-state-publisher`, `ros-noetic-image-transport-plugins`, and `ros-noetic-xacro`.
4.  Creates a temporary `builder` user with `sudo` privileges, as the ZED SDK installer requires being run by a non-root user.
5.  Gives the `builder` user ownership of the `/usr/local/zed` directory.
6.  Switches to the `builder` user to download and run the ZED SDK installer.
7.  Switches back to the `root` user.
8.  Creates a catkin workspace.
9.  Copies the `zed-ros-wrapper` source code into the workspace.
10. Clones the `zed-ros-interfaces` repository from GitHub into the workspace.
11. Builds the `zed_interfaces` package.
12. Installs the ROS dependencies for the project.
13. Builds the catkin workspace, providing the path to the ZED SDK's CMake files.
14. Sets up an entrypoint to source the ROS environment.

## 4. Usage

To build the Docker image:

```bash
docker build -t zed-ros-wrapper .
```

To run the Docker container:

```bash
docker run -d --runtime=nvidia --gpus all --privileged --network=host --ipc=host -v /dev:/dev zed-ros-wrapper
```

## 5. Docker Configuration for NVIDIA Runtime (if needed)

If you encounter the error "unknown or invalid runtime name: nvidia" when running the Docker container, it means Docker is not aware of the NVIDIA runtime. You need to create or modify the `/etc/docker/daemon.json` file on your host machine.

1.  **Create/Edit `daemon.json`**: Open a terminal and run the following command to create or update the file:

    ```bash
sudo tee /etc/docker/daemon.json <<EOF
{
    "runtimes": {
        "nvidia": {
            "path": "/usr/bin/nvidia-container-runtime",
            "runtimeArgs": []
        }
    }
}
EOF
    ```

2.  **Restart Docker Daemon**: After modifying `daemon.json`, you must restart the Docker daemon for the changes to take effect:

    ```bash
sudo systemctl restart docker
    ```

After these steps, you should be able to run the Docker container successfully.