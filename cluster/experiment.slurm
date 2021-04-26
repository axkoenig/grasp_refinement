#!/bin/bash
#SBATCH --job-name=experiment
#SBATCH --partition=shared
#SBATCH --mail-type=END,FAIL
#SBATCH --mail-user=awckoenig@gmail.com
#SBATCH -N 1                          
#SBATCH -n 1                          
#SBATCH -c 4
#SBATCH --mem=16000       
#SBATCH -t 0-04:00              
# ====
# you must set the following variables using the --export flag when running this slurm script. view experiments.sh for an example.
# ====
# FRAMEWORK
# TIME_STEPS
# LOG_NAME
# CLUSTER_PATH
# HOME_DIR
# MAX_EP_LEN
# X_ERROR_MIN
# X_ERROR_MAX
# Y_ERROR_MIN
# Y_ERROR_MAX
# Z_ERROR_MIN
# Z_ERROR_MAX
# ==== 
echo "================="
echo "Starting training"
echo "================="
read -r -d '' TRAIN_EXEC << EOM
. /opt/ros/noetic/setup.sh
. $HOME_DIR/overlay/work/catkin_ws/devel/setup.sh
roslaunch agent training.launch framework:=${FRAMEWORK} output_dir:=$HOME_DIR/output/agents log_name:=${LOG_NAME} time_steps:=${TIME_STEPS} max_ep_len:=${MAX_EP_LEN} x_error_min:=${X_ERROR_MIN} y_error_min:=${Y_ERROR_MIN} z_error_min:=${Z_ERROR_MIN} x_error_max:=${X_ERROR_MAX} y_error_max:=${Y_ERROR_MAX} z_error_max:=${Z_ERROR_MAX} 
EOM
echo -e "Executing the following script ...\n$TRAIN_EXEC"
singularity exec -e --bind $HOME_DIR/output/:/output $HOME_DIR/gazebo_ros.img /bin/sh -c "$TRAIN_EXEC"
echo "Done! Have a nice day."
