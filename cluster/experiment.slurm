#!/bin/bash
#SBATCH --job-name=experiment
#SBATCH --partition=serial_requeue
#SBATCH --mail-type=END,FAIL
#SBATCH --mail-user=awckoenig@gmail.com
#SBATCH -N 1                          
#SBATCH -n 1                          
#SBATCH -c 4
#SBATCH --mem=16000       
#SBATCH -t 0-15:00              
# ====
# you must set the following variables using the --export flag when running this slurm script. view experiments.sh for an example.
# ====
# FRAMEWORK
# TIME_STEPS
# LOG_NAME
# CLUSTER_PATH
# HOME_DIR
# X_ERROR_MIN
# X_ERROR_MAX
# Y_ERROR_MIN
# Y_ERROR_MAX
# Z_ERROR_MIN
# Z_ERROR_MAX
# ROLL_ERROR_MIN
# ROLL_ERROR_MAX
# PITCH_ERROR_MIN
# PITCH_ERROR_MAX
# YAW_ERROR_MIN
# YAW_ERROR_MAX
# W_BINARY_REW
# W_EPS_TORQUE
# W_DELTA
# SEED
# ROS_PORT
# GAZEBO_PORT
# ==== 
LOG_FILE_SIM=$HOME_DIR/output/slurm_logs/sim_${LOG_NAME}.txt
LOG_FILE_AGENT=$HOME_DIR/output/slurm_logs/agent_${LOG_NAME}.txt
echo "================================="
echo "Starting simulation in background"
echo "================================="
read -r -d '' SIM_EXEC << EOM
. /opt/ros/noetic/setup.sh
. $HOME_DIR/overlay/work/catkin_ws/devel/setup.sh
export ROS_MASTER_URI=http://localhost:$ROS_PORT
export GAZEBO_MASTER_URI=http://localhost:$GAZEBO_PORT
script -c "roslaunch description reflex.launch gui:=false" $LOG_FILE_SIM
EOM
echo -e "Executing the following script ...\n$SIM_EXEC"
screen -S simulation -d -m singularity exec -e --bind $HOME_DIR/output/:/output $HOME_DIR/gazebo_ros.img /bin/sh -c "$SIM_EXEC"
echo "Sleeping 10s to give simulation time to start"
sleep 10 
echo "Sleeping done."
echo "========================="
echo "Starting training session"
echo "========================="
read -r -d '' AGENT_EXEC << EOM
. /opt/ros/noetic/setup.sh
. $HOME_DIR/overlay/work/catkin_ws/devel/setup.sh
export ROS_MASTER_URI=http://localhost:$ROS_PORT
export GAZEBO_MASTER_URI=http://localhost:$GAZEBO_PORT
script -c "python3 $HOME_DIR/overlay/work/catkin_ws/src/grasp_refinement/agent/src/main.py --framework=${FRAMEWORK} --seed=${SEED} --output_dir=$HOME_DIR/output/agents --log_name=${LOG_NAME} --time_steps=${TIME_STEPS} --w_binary_rew ${W_BINARY_REW} --w_eps_torque ${W_EPS_TORQUE} --w_delta ${W_DELTA} --x_error_min ${X_ERROR_MIN} --x_error_max ${X_ERROR_MAX} --y_error_min ${Y_ERROR_MIN} --y_error_max ${Y_ERROR_MAX} --z_error_min ${Z_ERROR_MIN} --z_error_max ${Z_ERROR_MAX} --roll_error_min ${ROLL_ERROR_MIN} --roll_error_max ${ROLL_ERROR_MAX} --pitch_error_min ${PITCH_ERROR_MIN} --pitch_error_max ${PITCH_ERROR_MAX} --yaw_error_min ${YAW_ERROR_MIN} --yaw_error_max ${YAW_ERROR_MAX}" $LOG_FILE_AGENT
EOM
echo -e "Executing the following script ...\n$AGENT_EXEC"
singularity exec -e --bind $HOME_DIR/output/:/output $HOME_DIR/gazebo_ros.img /bin/sh -c "$AGENT_EXEC"
echo "==================="
echo "Killing simulation"
echo "==================="
screen -X -S simulation quit
echo "Done! Have a nice day."
