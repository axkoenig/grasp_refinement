# Tactile Grasp Refinement using Deep Reinforcement Learning and Analytic Grasp Stability Metrics

<img src="docs/grasp_refinement.png"/>

This is the offical code repository for the publication "[Tactile Grasp Refinement using Deep Reinforcement Learning and Analytic Grasp Stability Metrics](https://arxiv.org/abs/2109.11234)" which is currently under review.

# Citation

```bash
@misc{koenig2021tactile,
      title={Tactile Grasp Refinement using Deep Reinforcement Learning and Analytic Grasp Stability Metrics}, 
      author={Alexander Koenig and Zixi Liu and Lucas Janson and Robert Howe},
      year={2021},
      eprint={2109.11234},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
}
```

## Installation

0. Disclaimer: the below steps assume you have a fresh installation of Ubuntu 20.04.
1. Install ROS Noetic by following [these](http://wiki.ros.org/noetic/Installation/Ubuntu) steps.
2. Clone this repository into a new catkin workspace.
```bash 
# Init new catkin workspace
mkdir ~/catkin_ws/src -p
cd ~/catkin_ws/src
catkin_init_workspace
# Clone this repository with its submodules
git clone --recursive https://github.com/axkoenig/grasp_refinement.git
```
3. This paper uses the [Reflex Stack](https://github.com/axkoenig/reflex_stack), a software module that simulates the robotic hand and comes with various useful tools for real-time grasp analysis. The simulator runs Gazebo 11 and DART 6. To run Gazebo with the DART physics engine, you must build Gazebo from source. Running the shell script does this for you. 
```bash 
cd ~/catkin_ws/src/grasp_refinement/reflex_stack/shell
sudo ./install_gazebo_dart.sh
```
4. Now that you have all the required dependencies you can install the software. 
```bash 
# Build software
cd ~/catkin_ws
catkin_make
# Source workspace and add to your bashrc
source ~/catkin_ws/devel/setup.bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```
5. Check if everything works by firing up the [Reflex Stack](https://github.com/axkoenig/reflex_stack) simulator in a new terminal.
```bash 
roslaunch description reflex.launch run_keyboard_teleop_nodes:=true
```

## Train an Agent

You can train an agent with a one-liner. 

```bash
cd ~/catkin_ws/src/grasp_refinement/agent/src
python main.py --gui=1 --reward_framework=1 --force_framework=1 --log_name=i_love_robots
```

## Enjoy a pre-trained Agent

Enjoy one of the pre-trained agents, or one of the agents you trained yourself. The 

```bash
python main.py --train=0 test_model_path=~/catkin_ws/src/grasp_refinement/trained_agents/epsilon+delta_full.zip --all_test_cases=0 --gui=1 --reward_framework=1 --force_framework=1 --log_name=i_love_robots
```