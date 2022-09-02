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

FROM ros:$ROSDISTRO

ARG ROSDISTRO
ARG WORKSPACE

## Use supporting images for multi-stage build

# Enable this COPY if you want to use tf2_ros compiled for Python 3
COPY --from=python3-tf2_ros \
    /ws/install \
    /opt/ros/$ROSDISTRO

## Create svea workspace

COPY . $WORKSPACE
WORKDIR $WORKSPACE

## Install dependencies

ARG DEBIAN_FRONTEND=noninteractive
RUN cp -f $WORKSPACE/entrypoint /ros_entrypoint.sh && \
    apt-get update -y && \
    apt-get upgrade -y && \
    apt-get install --no-install-recommends -y \
        apt-utils \
        git vim nano curl \
        python-pip \
        python-numpy \
        python-matplotlib \
        python-tk \
        python3-pip \
        python3-numpy \
        python3-tk \
        python3-matplotlib \
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
