#!/bin/bash
#SBATCH --job-name=experiment
#SBATCH --partition=shared
#SBATCH --mail-type=END,FAIL
#SBATCH --mail-user=awckoenig@gmail.com
#SBATCH -N=1                          
#SBATCH -n=8                          
#SBATCH --mem=16000       
#SBATCH --time=00:04:00           
#SBATCH --output=logs_slurm/job_%j.log 
echo "================================="
echo "Starting simulation in background"
echo "================================="
CLUSTER_PATH=$HOME/overlay/work/catkin_ws/src/grasp_refinement/cluster
IMG_PATH=$HOME
FRAMEWORK=1
screen -S simulation -d -m singularity exec -e --overlay $IMG_PATH/overlay.img --bind $SCRATCH/howe_lab_seas/Users/akoenig/output/:/output $IMG_PATH/gazebo_ros.img /bin/sh $CLUSTER_PATH/run_sim.sh
echo "========================="
echo "Starting training session"
echo "========================="
singularity exec -e --overlay $IMG_PATH/overlay.img --bind $SCRATCH/howe_lab_seas/Users/akoenig/output/:/output $IMG_PATH/gazebo_ros.img /bin/sh $CLUSTER_PATH/run_agent.sh  \
    --experiment_id=%j          \
    --x_error=0.04              \
    --y_error=0.04              \
    --z_error=0.04              \
    --time_steps=10000          \
    --max_ep_len=15             \
    --framework=$FRAMEWORK      \  # this var is set in experiments.sh
    --log_name=$LOG_NAME        \  # this var is set in experiments.sh
echo "==================="
echo "Killing simulation"
echo "==================="
screen -X -S simulation quit