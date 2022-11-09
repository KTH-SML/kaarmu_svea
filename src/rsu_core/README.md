# RSU Core Package

### Quicklinks

- [SVEA website](https://svea.eecs.kth.se)
- [SVEA repository](https://github.com/KTH-SML/svea)


## Description

This repository provides the core functionality of a road-side unit (RSU) for 
development and research applications. The software is built for tight
integration with the SVEA platform, or similar ROS projects that wants to 
utilize a RSU. For this purpose, the RSU code is written in python as a ROS 
package.


## Installation

Before continuing to the next sections, consider taking some time to read up on
two important concepts for this code base: the **Robotic Operating System (ROS)**
and **Object Oriented Programming (OOP)**.

To read up on ROS, check out the
[ROS Start Guide](http://wiki.ros.org/ROS/StartGuide). However, do not spend
too much time diving into the guide. The structure and tutorials are not very
intuitive, but glossing over them will give a sense of what ROS is and how you
are meant to use it. The rest of the learning curve is overcome by trying it out
yourself.

To read up on OOP, check out Real Python's
[introduction on OOP](https://realpython.com/python3-object-oriented-programming/).

### System Requirements

- Ubuntu 18.04
- ROS Melodic 
- Python 2.7
- Python 3.6
- (optional) CUDA enabled GPU, not tested without
- (optional) ZED stereo camera, requires ZED SDK and CUDA enabled GPU

When using Ubuntu 18.04, all the other system requirements follow. The only ROS 
distribution for Ubuntu 18.04 is ROS Melodic. Python 2.7 and Python 3.6 should
be installed already on a Ubuntu 18.04 system. After installing ROS, Python 2.7 
will become the default Python version, and Python 3.6 won't work with ROS. You
must manually enable this, read more in the next section.

When developing it is highly recommended to use this environment on your local 
system. However, if that is not possible you can use a virtual machine. This
will affect the performance. 

When deploying to a "production" device, i.e. for demonstrations or experiments,
you can set up a docker container as well. Just be sure to use nvidia's docker
runtime for access to the GPU.


### Installing the libraray

*NOTE:* This installation will use `catkin build` instead of `catkin_make`.

1. Go to your catkin workspace's `src` directory
2. Clone the package and the ZED dependency (optional)
3. If you use an old version of JetPack (on nvidia jetsons), an old version of 
   CUDA or maybe because of something else, you might need to use a specific 
   version of zed-ros-wrapper. You can read more about this on Stereolabs
   website. If you are using a Jetson then it might be difficult to confirm the 
   JetPack version you are running. If available, use the `jtop` command.
   - [`zed-ros-wrapper` website](https://www.stereolabs.com/docs/ros/)
   - [`zed-ros-wrapper` repository](https://github.com/stereolabs/zed-ros-wrapper)
   - [ZED SDK on Linux](https://www.stereolabs.com/docs/installation/linux/) 
   - [ZED SDK on Jetson](https://www.stereolabs.com/docs/installation/jetson/)
   - [ZED SDK Downloads](https://www.stereolabs.com/developers/release/) (Check legacy archive as well)
4. Install dependencies using `rosdep`
5. Build the workspace using `catkin build`
6. Optionally, if you want to automatically source the workspace at log in, then execute...


```bash
WORKSPACE=<workspace-path>

# (1) Go to your catkin workspace's `src` directory.
cd "$WORKSPACE/src"

# (2) Clone the package and the ZED dependency (optional)
git clone https://github.com/KTH-SML/rsu_core
git clone https://github.com/stereolabs/zed-ros-wrapper

# (3) Switch to the correct zed-wrapper version
cd zed-ros-wrapper
git checkout v2.x # Example for JetPack v4.2 

# (4) Install dependencies using `rosdep` 
cd "$WORKSPACE"
rosdep --from-paths src --ignore-src -r -y 

# (5) Build the workspace
catkin build
source devel/setup.bash 

# (6) Optionally, if you want to automatically source the workspace at log in, then execute...
echo "source '$WORKSPACE/devel/setup.bash' >> ~/.bashrc"
```


## Usage

