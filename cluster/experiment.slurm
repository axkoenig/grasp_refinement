#!/bin/bash
#SBATCH --job-name=experiment
#SBATCH --partition=shared
#SBATCH --mail-type=END,FAIL
#SBATCH --mail-user=awckoenig@gmail.com
#SBATCH -N 1
#SBATCH -n 1
#SBATCH -c 4
#SBATCH --mem=16000
#SBATCH -t 2-00:00
# ====
# you must set the following variables using the --export flag when running this slurm script. view experiments.sh for an example.
# ====
# REWARD_FRAMEWORK
# TIME_STEPS
# ALGORITHM
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
# W_EPS_TORQUE
# W_DELTA
# GRADIENT_STEPS
# TAU
# TRAIN_FREQ
# SEED
# ROS_PORT
# GAZEBO_PORT
# ====
LOG_FILE_SIM=$HOME_DIR/output/slurm_logs/sim_${LOG_NAME}.txt
LOG_FILE_AGENT=$HOME_DIR/output/slurm_logs/agent_${LOG_NAME}.txt
echo "========================="
echo "Starting training session"
echo "========================="
read -r -d '' AGENT_EXEC <<EOM
pip3 install stable-baselines3 --upgrade
. /usr/share/gazebo/setup.sh
. /opt/ros/noetic/setup.sh
. $HOME_DIR/overlay/work/catkin_ws/devel/setup.sh
export ROS_MASTER_URI=http://localhost:$ROS_PORT
export GAZEBO_MASTER_URI=http://localhost:$GAZEBO_PORT
script -c "python3 $HOME_DIR/overlay/work/catkin_ws/src/grasp_refinement/agent/src/main.py --eval_after_training=0 --gui=0 --reward_framework=${REWARD_FRAMEWORK} --batch_size=${BATCH_SIZE} --tau=${TAU} --learning_rate=${LEARNING_RATE} --force_framework=${FORCE_FRAMEWORK} --seed=${SEED} --algorithm=${ALGORITHM} --gradient_steps=${GRADIENT_STEPS} --train_freq=${TRAIN_FREQ} --output_dir=$HOME_DIR/output/agents --log_name=${LOG_NAME} --time_steps=${TIME_STEPS} --w_eps_torque ${W_EPS_TORQUE} --w_delta ${W_DELTA} --x_error_min ${X_ERROR_MIN} --x_error_max ${X_ERROR_MAX} --y_error_min ${Y_ERROR_MIN} --y_error_max ${Y_ERROR_MAX} --z_error_min ${Z_ERROR_MIN} --z_error_max ${Z_ERROR_MAX} --roll_error_min ${ROLL_ERROR_MIN} --roll_error_max ${ROLL_ERROR_MAX} --pitch_error_min ${PITCH_ERROR_MIN} --pitch_error_max ${PITCH_ERROR_MAX} --yaw_error_min ${YAW_ERROR_MIN} --yaw_error_max ${YAW_ERROR_MAX}" $LOG_FILE_AGENT
EOM
echo -e "Executing the following script ...\n$AGENT_EXEC"
singularity exec -e --bind $HOME_DIR/output/:/output $HOME_DIR/gazebo_ros.img /bin/sh -c "$AGENT_EXEC"
echo "========================="
echo "Starting testing session"
echo "========================="
LOG_FILE_AGENT=$HOME_DIR/output/slurm_logs/agent_${LOG_NAME}_testing.txt
read -r -d '' AGENT_EXEC <<EOM
pip3 install stable-baselines3 --upgrade
. /usr/share/gazebo/setup.sh
. /opt/ros/noetic/setup.sh
. $HOME_DIR/overlay/work/catkin_ws/devel/setup.sh
export ROS_MASTER_URI=http://localhost:$ROS_PORT
export GAZEBO_MASTER_URI=http://localhost:$GAZEBO_PORT
script -c "python3 $HOME_DIR/overlay/work/catkin_ws/src/grasp_refinement/agent/src/main.py --eval_after_training=0 --gui=0 --train=0 --test_model_path=${HOME_DIR}/output/agents/models/refinement/${LOG_NAME}/final_model.zip --reward_framework=${REWARD_FRAMEWORK} --batch_size=${BATCH_SIZE} --tau=${TAU} --learning_rate=${LEARNING_RATE} --force_framework=${FORCE_FRAMEWORK} --seed=${SEED} --algorithm=${ALGORITHM} --gradient_steps=${GRADIENT_STEPS} --train_freq=${TRAIN_FREQ} --output_dir=$HOME_DIR/output/agents --log_name=${LOG_NAME} --time_steps=${TIME_STEPS} --w_eps_torque ${W_EPS_TORQUE} --w_delta ${W_DELTA} --x_error_min ${X_ERROR_MIN} --x_error_max ${X_ERROR_MAX} --y_error_min ${Y_ERROR_MIN} --y_error_max ${Y_ERROR_MAX} --z_error_min ${Z_ERROR_MIN} --z_error_max ${Z_ERROR_MAX} --roll_error_min ${ROLL_ERROR_MIN} --roll_error_max ${ROLL_ERROR_MAX} --pitch_error_min ${PITCH_ERROR_MIN} --pitch_error_max ${PITCH_ERROR_MAX} --yaw_error_min ${YAW_ERROR_MIN} --yaw_error_max ${YAW_ERROR_MAX}" $LOG_FILE_AGENT
EOM
echo -e "Executing the following script ...\n$AGENT_EXEC"
singularity exec -e --bind $HOME_DIR/output/:/output $HOME_DIR/gazebo_ros.img /bin/sh -c "$AGENT_EXEC"
echo "Done! Have a nice day."
