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
NOW=$(date +"%m-%d-%Y-%T")
HOME_DIR=$SCRATCH/howe_lab_seas/Users/akoenig
CLUSTER_PATH=$HOME_DIR/overlay/work/catkin_ws/src/grasp_refinement/cluster
LOG_PATH=$HOME_DIR/output/slurm_logs
echo "Submitting ${NUM_EXPERIMENTS} experiments for each framework..."

# hyper-params for training
TIME_STEPS=20000
MAX_EP_LEN=15
X_ERROR_MAX=0.04
X_ERROR_MIN=-$X_ERROR_MAX
Y_ERROR_MAX=0.0
Y_ERROR_MIN=-$Y_ERROR_MAX
Z_ERROR_MAX=0.03
Z_ERROR_MIN=-$Z_ERROR_MAX

submit_job () {
    LOG_NAME=${NOW}_f${1}_id${2}
    echo "Submitting job with log name $LOG_NAME"
    sbatch --output=$LOG_PATH/$LOG_NAME.out --error=$LOG_PATH/$LOG_NAME.err --export=FRAMEWORK=${1},TIME_STEPS=${TIME_STEPS},LOG_NAME=${LOG_NAME},CLUSTER_PATH=$CLUSTER_PATH,HOME_DIR=$HOME_DIR,MAX_EP_LEN=$MAX_EP_LEN,X_ERROR_MAX=$X_ERROR_MAX,X_ERROR_MIN=$X_ERROR_MIN,Y_ERROR_MAX=$Y_ERROR_MAX,Y_ERROR_MIN=$Y_ERROR_MIN,Z_ERROR_MAX=$Z_ERROR_MAX,Z_ERROR_MIN=$Z_ERROR_MIN ${CLUSTER_PATH}/experiment.slurm
} 

for i in $(seq 1 $NUM_EXPERIMENTS); 
do 
submit_job "1" ${i} # framework 1
submit_job "2" ${i} # framework 2
done

echo "Done."
