# Dockerfile for SVEA image
#
# Author: Kaj Munhoz Arfvidsson

ARG ROSDISTRO
ARG WORKSPACE

#######################
## SUPPORTING IMAGES ##
#######################

## python3-tf2_ros ##

FROM ros:$ROSDISTRO AS python3-tf2_ros

ARG ROSDISTRO

ARG DEBIAN_FRONTEND=noninteractive

WORKDIR /ws

RUN apt-get update -y && \
    apt-get install -y --no-install-recommends \
        python3-pip \
        python3-catkin-tools \
        python3-empy \
        && \
    catkin config \
        --init \
        --mkdirs \
        --install \
        --extend /opt/ros/$ROSDISTRO \
        --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DPYTHON_EXECUTABLE=/usr/bin/python3 \
            -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m \
            -DPYTHON_LIBRARY=/usr/lib/$(uname -m)-linux-gnu/libpython3.6m.so \
        && \
    wstool init && \
    wstool set \
        -y src/geometry2 \
        --git https://github.com/ros/geometry2 \
        -v 0.6.5 \
        && \
    wstool up && \
    rosdep install \
        --from-paths src \
        --ignore-src \
        -q -y -r \
        && \
    catkin build tf2_ros

#####################
## SVEA BASE IMAGE ##
#####################

FROM dustynv/ros:$ROSDISTRO-ros-base-l4t-r32.7.1

ARG ROSDISTRO
ARG WORKSPACE

## Install ZED SDK

#This environment variable is needed to use the streaming features on Jetson inside a container
ARG DEBIAN_FRONTEND noninteractive
RUN apt-get update -y && \
    apt-get install -y --no-install-recommends \
        lsb-release \
        wget \
        less \
        udev \
        apt-transport-https \
        && \
    wget \
        -q --no-check-certificate \
        -O ZED_SDK_Linux_JP.run \
        https://download.stereolabs.com/zedsdk/3.7/l4t32.7/jetsons \
        && \
    chmod +x ZED_SDK_Linux_JP.run && \
    ./ZED_SDK_Linux_JP.run silent runtime_only && \
    rm -rf \
        ZED_SDK_Linux_JP.run \
        /usr/local/zed/resources/* \
        /var/lib/apt/lists/*

# ZED Python API
RUN apt-get update -y && \
    apt-get install -y --no-install-recommends \
        python3 \
        python3-pip \
        python3-dev \
        python3-setuptools \
        build-essential \
        && \
    wget \
        -O /usr/local/zed/get_python_api.py \
        https://download.stereolabs.com/zedsdk/pyzed \
        && \
    python3 /usr/local/zed/get_python_api.py && \
    python3 -m pip install cython wheel && \
    python3 -m pip install numpy *.whl && \
    apt-get remove -y --purge \
        build-essential \
        python3-dev \
        && \
    apt-get autoremove -y && \
    rm -rf \
        *.whl \
        /var/lib/apt/lists/*

# This symbolic link is needed to use the streaming features on Jetson inside a container
RUN ln -sf /usr/lib/aarch64-linux-gnu/tegra/libv4l2.so.0 /usr/lib/aarch64-linux-gnu/libv4l2.so

## Use supporting images for multi-stage build

# Enable this COPY if you want to use tf2_ros compiled for Python 3
COPY --from=python3-tf2_ros \
    /ws/install \
    /opt/ros/$ROSDISTRO

## Create svea workspace

COPY . $WORKSPACE
WORKDIR $WORKSPACE

## Install dependencies

RUN cp -f $WORKSPACE/entrypoint /ros_entrypoint.sh && \
    apt-get update -y && \
    apt-get upgrade -y && \
    apt-get install --no-install-recommends -y \
        apt-utils \
        git vim nano curl \
        python-pip \
        python-numpy \
        python-matplotlib \
        python-scipy \
        python-tk \
        python3-pip \
        python3-numpy \
        python3-tk \
        python3-matplotlib \
        python3-scipy \
        python3-catkin-tools \
        python3-rospkg-modules \
        && \
    python -m pip install -U pip && \
    python3 -m pip install -U pip && \
    rosdep update \
        --rosdistro $ROSDISTRO \
        && \
    rosdep install \
        --rosdistro $ROSDISTRO \
        --from-paths src \
        --ignore-src \
        -q -r -y

# Run catkin build on workspace.
# (improves build speeds later on)

RUN catkin config \
        --init \
        --mkdirs \
        --extend /opt/ros/$ROSDISTRO \
        > /dev/null \
        && \
    catkin build || \
    true

# Container entrypoint (executes util/entrypoint)
# bash is run by default when user starts

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
