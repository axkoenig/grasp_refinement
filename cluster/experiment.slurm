#!/bin/bash
#SBATCH --job-name=experiment
#SBATCH --partition=shared
#SBATCH --mail-type=END,FAIL
#SBATCH --mail-user=awckoenig@gmail.com
#SBATCH -N 1                          
#SBATCH -n 8                          
#SBATCH --mem=16000       
#SBATCH --time=00:04:00           
#SBATCH --output=$HOME/logs_slurm/job_%LOG_NAME.log 
echo "================================="
echo "Starting simulation in background"
echo "================================="
CLUSTER_PATH=$HOME/overlay/work/catkin_ws/src/grasp_refinement/cluster
IMG_PATH=$HOME
screen -S simulation -d -m singularity exec -e --bind $SCRATCH/howe_lab_seas/Users/akoenig/output/:/output $IMG_PATH/gazebo_ros.img /bin/sh $CLUSTER_PATH/run_sim.sh
echo "========================="
echo "Starting training session"
echo "========================="
EXEC_SCRIPT="$CLUSTER_PATH/run_agent.sh --x_error 0.04              \
                                        --y_error 0.04              \
                                        --z_error 0.04              \
                                        --time_steps $TIME_STEPS    \
                                        --max_ep_len 15             \
                                        --framework $FRAMEWORK      \
                                        --log_name $LOG_NAME        \
                                        "
singularity exec -e --bind $SCRATCH/howe_lab_seas/Users/akoenig/output/:/output $IMG_PATH/gazebo_ros.img /bin/sh -c "$EXEC_SCRIPT"
echo "==================="
echo "Killing simulation"
echo "==================="
screen -X -S simulation quit
