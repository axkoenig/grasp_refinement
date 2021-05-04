# Teleoperated Grasp Refinement

[![Build Status](https://travis-ci.com/axkoenig/grasp_refinement.svg?token=KeJradpJgXCJqZfQ8pwB&branch=main)](https://travis-ci.com/axkoenig/grasp_refinement)

## Run Software with Singularity

Install singularity on your local machine (remote machine should have singularity installed) and follow these steps.

On your local machine. 

```bash
# 1. Build the singularity recipe to get a clean image with all required dependencies (e.g., Gazebo 11, ROS Noetic, RL packages)
sudo singularity build gazebo_ros.img singularity.recipe 

# 2. Create an overlay directory and build an overlay image
mkdir -p overlay/upper overlay/work
dd if=/dev/zero of=overlay.img bs=1M count=500 && mkfs.ext3 -d overlay overlay.img

# 3. Upload both images to your research cluster
sudo sftp akoenig@login.rc.fas.harvard.edu
put -r /home/parallels/catkin_ws/src/grasp_refinement/gazebo_ros.img
put -r /home/parallels/catkin_ws/src/grasp_refinement/overlay.img

``` 

On your remote machine do these steps. 

```bash
# create the same overlay directories as on local machine
mkdir -p overlay/upper overlay/work

# fire up your simulation container with the writable overlay to build the software
singularity shell --overlay overlay.img gazebo_ros.img

# create catkin workspace inside the overlay container and init a catkin workspace
mkdir ~/overlay/work/catkin_ws/src -p
cd ~/overlay/work/catkin_ws/src
. /opt/ros/noetic/setup.sh
catkin_init_workspace

# Clone your favourite branch of this repository
cd ~/overlay/work/catkin_ws/src
git clone -b feature/multiple_objects --recursive git@github.com:axkoenig/grasp_refinement.git 
cd ~/overlay/work/catkin_ws
catkin_make

# To test if everything works
. ~/overlay/work/catkin_ws/devel/setup.sh
roslaunch description reflex.launch gui:=false

``` 

To train on the cluster.

```bash
.cluster/experiments.sh -n 1   # start experiments
squeue -u akoenig              # viel all jobs
scancel -u akoenig             # cancel all jobs
screen -r       # reattach to screen
``` 




## Install Software 
1. Install Gazebo 11 with the DART 6 physics engine (DART has proven to work better at simulating grasping than the default physics engine (ODE)). You will need to build Gazebo from source to work with DART. Follow the [official instructions](http://gazebosim.org/tutorials?tut=install_from_source&cat=install) for doing so.

2. Install ROS noetic by following the [official instructions](https://wiki.ros.org/noetic/Installation/Ubuntu). The ```ros-noetic-desktop``` is recommended. The ```ros-noetic-desktop-full``` would also install Gazebo and this might conflict with the installation from step 1. 

3. Install ROS controllers for robot control and the Gazebo ROS integration.
```bash
sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers
sudo apt-get install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control
``` 

4. Setup new catkin workspace.

```bash
mkdir ~/catkin_ws/src -p
cd ~/catkin_ws/src
catkin_init_workspace
```

5. Clone repository with its submodules into ```src``` folder.

```bash
git clone --recursive https://github.com/axkoenig/grasp_refinement.git
```

6. Build workspace.

```bash
cd ~/catkin_ws
catkin_make
```

7. Source this workspace and, if you like, add it to your ```.bashrc```.

```bash
source ~/catkin_ws/devel/setup.bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

## Acknowledgements

- The robot description package was originally based on the ```ll4ma_robots_description``` package by the [Utah Learning Lab for Manipulation Autonomy](https://bitbucket.org/robot-learning/ll4ma_robots_description/src/main/). The parts that were unnecessary for this project were removed. The Reflex robotic hand was also modified to allow for basic actuation of the distal flexure.
- Developer of [teleop_twist_keyboard.cpp](https://github.com/methylDragon/teleop_twist_keyboard_cpp/blob/master/src/teleop_twist_keyboard.cpp) for making code of non-blocking keyboard input public. Used in my keyboard teleoperation node. 