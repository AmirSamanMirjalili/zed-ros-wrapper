# 1. Start from a CUDA development base image
FROM nvidia/cuda:12.1.0-devel-ubuntu20.04

# Set non-interactive installation
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

# 2. Install system dependencies and add the ROS repository
RUN apt-get update && apt-get install -y --no-install-recommends \
    gnupg2 \
    lsb-release \
    curl \
    wget \
    sudo \
    && sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
    && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
    && rm -rf /var/lib/apt/lists/*

# 3. Install ROS Noetic and all required dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-rosdep \
    zstd \
    libopencv-dev \
    libncurses5-dev \
    libudev-dev \
    ros-noetic-ros-base \
    ros-noetic-image-transport \
    ros-noetic-rosconsole \
    ros-noetic-sensor-msgs \
    ros-noetic-stereo-msgs \
    ros-noetic-std-msgs \
    ros-noetic-std-srvs \
    ros-noetic-message-filters \
    ros-noetic-tf2-ros \
    ros-noetic-nodelet \
    ros-noetic-tf2-geometry-msgs \
    ros-noetic-message-generation \
    ros-noetic-diagnostic-updater \
    ros-noetic-dynamic-reconfigure \
    ros-noetic-robot-state-publisher \
    ros-noetic-image-transport-plugins \
    less \
    file \
    libblas-dev \
    liblapack-dev     ros-noetic-xacro     libusb-1.0-0-dev &&     rm -rf /var/lib/apt/lists/*

# 4. Create a non-root user to run the ZED SDK installer
RUN useradd -m builder && \
    echo "builder:builder" | chpasswd && \
    adduser builder sudo
RUN echo "builder ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Pre-create the ZED installation directory
RUN mkdir -p /usr/local/zed && chown -R builder:builder /usr/local/zed

# Switch to the non-root user to run the installer, but have it install to a local directory
USER builder
WORKDIR /home/builder

# 5. Install ZED SDK to a local directory
RUN mkdir /home/builder/zed_install && \
    wget -q -O ZED_SDK.run https://download.stereolabs.com/zedsdk/4.1/cu121/ubuntu20 && \
    chmod +x ZED_SDK.run && \
    ./ZED_SDK.run --accept --quiet --target /home/builder/zed_install -- -s && \
    rm ZED_SDK.run

# Switch back to root user
USER root

# Copy the installed files to the final location and clean up
RUN cp -a /home/builder/zed_install/* /usr/local/zed/ && \
    rm -rf /home/builder/zed_install

RUN ldconfig

# 6. Create and build the catkin workspace
WORKDIR /catkin_ws
RUN mkdir -p src
COPY --chown=root:root . src/zed-ros-wrapper
RUN . /opt/ros/noetic/setup.sh && \
    rosdep init && rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y --os ubuntu:focal
RUN . /opt/ros/noetic/setup.sh && \
    catkin_make -DCMAKE_BUILD_TYPE=Release -DZED_DIR=/usr/local/zed/cmake

# 7. Set up the entrypoint
COPY ./docker-entrypoint.sh /
RUN chmod +x /docker-entrypoint.sh
ENTRYPOINT ["/docker-entrypoint.sh"]
CMD ["bash"] 