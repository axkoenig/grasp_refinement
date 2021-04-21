#!/bin/bash
#SBATCH --job-name=train_run
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
cd ~
CLUSTER_PATH=overlay/work/catkin_ws/src/grasp_refinement/cluster/
screen -S simulation -d -m singularity exec -e --bind $SCRATCH/howe_lab_seas/Users/akoenig/output/:/output gazebo_ros.img /bin/sh $CLUSTER_PATH/run_sim.sh --overlay overlay.img
echo "========================="
echo "Starting training session"
echo "========================="
singularity exec -e --bind $SCRATCH/howe_lab_seas/Users/akoenig/output/:/output gazebo_ros.img /bin/sh $CLUSTER_PATH/run_agent.sh --overlay overlay.img
echo "==================="
echo "Killing simulation"
echo "==================="
screen -X -S simulation quit