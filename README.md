# Teleoperated Grasp Refinement
Workspace for my master thesis on Teleoperated Grasp Refinement at the Harvard Biorobotics Lab

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

- Gazebo dies down with ```[gazebo-2] process has died```, then run ```killall gzserver``` before restarting Gazebo. See this [issue](https://answers.gazebosim.org//question/4153/gazebo-crashes-immediately-using-roslaunch-after-installing-gazebo-ros-packages/).

- Gazebo in waiting state --> rosrun gazebo_ros gazebo
