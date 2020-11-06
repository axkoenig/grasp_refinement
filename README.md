# Teleoperated Grasp Refinement

[![Build Status](https://travis-ci.com/axkoenig/grasp_refinement.svg?token=KeJradpJgXCJqZfQ8pwB&branch=main)](https://travis-ci.com/axkoenig/grasp_refinement)

## Install Software 

1. Install ROS noetic by following the [official instructions](https://wiki.ros.org/noetic/Installation/Ubuntu)

2. Setup new catkin workspace

```bash
mkdir ~/catkin_ws/src -p
cd ~/catkin_ws/src
catkin_init_workspace
```

3. Clone repository into ```src``` folder

```bash
git clone https://github.com/axkoenig/grasp_refinement.git
```

4. Build workspace

```bash
cd ~/catkin_ws
catkin_make
```

5. Source this workspace and, if you like, add it to your ```.bashrc```

```bash
source ~/catkin_ws/devel/setup.bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

## Troubleshooting

- Problems with Gazebo dies down with ```[gazebo-2] process has died```, then run ```killall gzserver``` before restarting Gazebo. See this [issue](https://answers.gazebosim.org//question/4153/gazebo-crashes-immediately-using-roslaunch-after-installing-gazebo-ros-packages/).

## Acknowledgements

- The robot description package was based on the ```ll4ma_robots_description``` package by the [Utah Learning Lab for Manipulation Autonomy](https://bitbucket.org/robot-learning/ll4ma_robots_description/src/main/). The parts that were unnecessary for this project were removed. The Reflex robotic hand was also modified to allow for basic actuation of the distal flexure.