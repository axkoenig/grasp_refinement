#!/bin/bash
# parse arguments
while [[ "$#" -gt 0 ]]
do
case $1 in
    -n|--num_experiments)
    NUM_EXPERIMENTS=$2
    ;;
esac
shift
done

# runs multiple experiments in parallel
NOW=$(date +"%m-%d-%Y/%T")
TIME_STEPS=10000
CLUSTER_PATH=$HOME/overlay/work/catkin_ws/src/grasp_refinement/cluster
echo "Submitting ${NUM_EXPERIMENTS} experiments for each framework..."

submit_job () {
    LOG_NAME=${NOW}_f${1}_id${i}
    echo "Submitting job with log name $LOG_NAME"
    sbatch --export=FRAMEWORK=${1},TIME_STEPS=${TIME_STEPS} --export=LOG_NAME=${LOG_NAME} ${CLUSTER_PATH}/experiment.slurm
} 

for i in $(seq 1 $NUM_EXPERIMENTS); 
do 
submit_job "1"  # framework 1
submit_job "2"  # framework 2
done

echo "Done."
