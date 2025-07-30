# ZED ROS Wrapper Docker Setup

This document outlines the requirements and steps to containerize the ZED ROS wrapper using Docker, including troubleshooting guidance based on common issues.

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
- **NVIDIA Video Codec Libraries**: `libnvidia-decode-535` and `libnvidia-encode-535` to prevent runtime errors with ZED SDK.
- **ZED SDK**: Version 4.1 for Ubuntu 20.04. The SDK will be downloaded and installed within the Docker image.
- **ROS Dependencies**: All necessary ROS packages required by the `zed-ros-wrapper` will be installed, including `robot_state_publisher`, `image_transport_plugins`, and `xacro`.
- **Project Code**: The project source code will be copied into the image and built using `catkin_make`.

## 3. Dockerfile

The `Dockerfile` performs the following steps:

1.  Starts from the `nvidia/cuda:12.1.0-devel-ubuntu20.04` base image.
2.  Installs system dependencies, including `sudo`, `less`, `file`, `libblas-dev`, `liblapack-dev`, `libusb-1.0-0-dev`, and the `zstd` decompression tool.
3.  Installs additional ROS packages: `ros-noetic-robot-state-publisher`, `ros-noetic-image-transport-plugins`, and `ros-noetic-xacro`.
4.  Installs NVIDIA video codec libraries to prevent runtime errors.
5.  Creates a temporary `builder` user with `sudo` privileges, as the ZED SDK installer requires being run by a non-root user.
6.  Gives the `builder` user ownership of the `/usr/local/zed` directory.
7.  Switches to the `builder` user to download and run the ZED SDK installer.
8.  Switches back to the `root` user and updates library cache with `ldconfig`.
9.  Creates a catkin workspace.
10. Copies the `zed-ros-wrapper` source code into the workspace.
11. Clones the `zed-ros-interfaces` repository from GitHub into the workspace.
12. Initializes rosdep and installs ROS dependencies before building.
13. Builds the `zed_interfaces` package first.
14. Builds the complete catkin workspace, providing the path to the ZED SDK's CMake files.
15. Updates library cache again to ensure all libraries are properly linked.
16. Sets up an entrypoint to source the ROS environment.

## 4. Building the Docker Image

To build the Docker image:

```bash
docker build -t zed-ros-wrapper .
```

**Note**: The build process may take 10-15 minutes depending on your internet connection and system performance.

## 5. Running the Docker Container

### Interactive Mode (Recommended for Development)
```bash
docker run --rm -it --gpus all --privileged --network=host --ipc=host -v /dev:/dev zed-ros-wrapper
```

### Detached Mode (For Background Services)
```bash
docker run -d --gpus all --privileged --network=host --ipc=host -v /dev:/dev zed-ros-wrapper
```

### Command Line Options Explained
- `--rm`: Automatically remove the container when it exits
- `-it`: Interactive mode with TTY (allows you to interact with the container)
- `-d`: Detached mode (runs in background)
- `--gpus all`: Enable GPU access for NVIDIA CUDA
- `--privileged`: Required for hardware access (cameras, USB devices)
- `--network=host`: Use host networking for ROS communication
- `--ipc=host`: Share IPC namespace for better performance
- `-v /dev:/dev`: Mount device directory for camera access

## 6. Using the ZED Camera

Once inside the Docker container, you can launch the ZED camera with:

```bash
# For ZED 2 (most common)
roslaunch zed_wrapper zed2.launch

# For other ZED models
roslaunch zed_wrapper zed.launch      # Original ZED
roslaunch zed_wrapper zed2i.launch    # ZED 2i  
roslaunch zed_wrapper zedm.launch     # ZED Mini
roslaunch zed_wrapper zedx.launch     # ZED X
roslaunch zed_wrapper zedxm.launch    # ZED X Mini

# Without TF transforms
roslaunch zed_wrapper zed_no_tf.launch
```

### Expected Behavior
- **With Camera Connected**: You should see initialization messages and the camera will start publishing topics.
- **Without Camera Connected**: You will see "CAMERA NOT DETECTED" messages every 2 seconds. This is normal behavior - the wrapper is searching for a camera.

## 7. Troubleshooting

### Common Issues and Solutions

#### 7.1 "Cannot locate node of type [zed_wrapper_node]"
**Cause**: The zed_wrapper package wasn't built properly.

**Solution**: Rebuild the workspace inside the container:
```bash
cd /catkin_ws
catkin_make --only-pkg-with-deps zed_wrapper
```

#### 7.2 "Could not find library corresponding to plugin zed_nodelets/ZEDWrapperNodelet"
**Cause**: The ZED nodelets library wasn't built.

**Solution**: Rebuild the complete workspace:
```bash
cd /catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release -DZED_DIR=/usr/local/zed/cmake
```

#### 7.3 "libnvcuvid.so.1: cannot open shared object file"
**Cause**: NVIDIA video codec libraries are not properly linked.

**Solution**: Update the library cache:
```bash
ldconfig
```

#### 7.4 "CAMERA NOT DETECTED"
**Causes and Solutions**:
- **No camera connected**: Connect your ZED camera via USB
- **Insufficient permissions**: Ensure the container has `--privileged` flag
- **Wrong camera model**: Try a different launch file (zed.launch, zed2i.launch, etc.)
- **USB connectivity**: Check `ls /dev/video*` to see if camera devices are detected

#### 7.5 "unknown or invalid runtime name: nvidia"
**Cause**: Docker doesn't recognize the NVIDIA runtime.

**Solution**: Configure Docker daemon (see section 8 below).

#### 7.6 Build Failures During Docker Image Creation
**Cause**: Network issues or dependency conflicts.

**Solutions**:
- Retry the build: `docker build --no-cache -t zed-ros-wrapper .`
- Check internet connectivity
- Ensure sufficient disk space (at least 10GB free)

## 8. Docker Configuration for NVIDIA Runtime (if needed)

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

## 9. Verifying the Setup

To verify everything is working correctly:

1. **Check ROS Environment**:
   ```bash
   echo $ROS_PACKAGE_PATH
   rospack find zed_wrapper
   ```

2. **Check ZED SDK Installation**:
   ```bash
   ls /usr/local/zed/
   ```

3. **Check Available Topics** (after launching):
   ```bash
   rostopic list | grep zed
   ```

4. **Check Camera Detection**:
   ```bash
   ls /dev/video*
   ```

## 10. Performance Notes

- The container requires significant resources due to the ZED SDK and ROS requirements
- GPU access is essential for optimal performance
- Network mode `host` is required for proper ROS communication
- The build process downloads several GB of data

## 11. Development Workflow

For active development, you may want to mount your source code:

```bash
docker run --rm -it --gpus all --privileged --network=host --ipc=host \
  -v /dev:/dev \
  -v $(pwd):/workspace/src/zed-ros-wrapper \
  zed-ros-wrapper
```

Then rebuild inside the container when you make changes:
```bash
cd /catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release -DZED_DIR=/usr/local/zed/cmake
```