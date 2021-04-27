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
echo "================================="
echo "Starting simulation in background"
echo "================================="
read -r -d '' SIM_EXEC << EOM
. /opt/ros/noetic/setup.sh
. $HOME_DIR/overlay/work/catkin_ws/devel/setup.sh
roslaunch description reflex.launch gui:=false
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
python3 $HOME_DIR/overlay/work/catkin_ws/src/grasp_refinement/agent/src/main.py --framework=${FRAMEWORK} --output_dir=$HOME_DIR/output/agents --log_name=${LOG_NAME} --time_steps=${TIME_STEPS} --max_ep_len=${MAX_EP_LEN} --x_error ${X_ERROR_MIN} ${X_ERROR_MAX} --y_error ${Y_ERROR_MIN} ${Y_ERROR_MAX} --z_error ${Z_ERROR_MIN} ${Z_ERROR_MAX}
EOM
echo -e "Executing the following script ...\n$AGENT_EXEC"
singularity exec -e --bind $HOME_DIR/output/:/output $HOME_DIR/gazebo_ros.img /bin/sh -c "$AGENT_EXEC"
echo "==================="
echo "Killing simulation"
echo "==================="
screen -X -S simulation quit
echo "Done! Have a nice day."