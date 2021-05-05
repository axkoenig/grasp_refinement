#!/bin/bash

#### USAGE
# ./experiments.sh -n 10 -c 05May_Test
####

# parse arguments
while [[ "$#" -gt 0 ]]
do
case $1 in
    -n|--num_experiments)
    NUM_EXPERIMENTS=$2
    ;;
esac
case $2 in
    -c|--comment)
    COMMENT=$3
    ;;
esac
shift
done

# runs multiple experiments in parallel
HOME_DIR=$SCRATCH/howe_lab_seas/Users/akoenig
CLUSTER_PATH=$HOME_DIR/overlay/work/catkin_ws/src/grasp_refinement/cluster
LOG_PATH=$HOME_DIR/output/slurm_logs
echo "Submitting ${NUM_EXPERIMENTS} experiments for each framework..."

# hyper-params for training
TIME_STEPS=3000
MAX_EP_LEN=15

# translational error
X_ERROR_MAX=0.04
X_ERROR_MIN=-$X_ERROR_MAX
Y_ERROR_MAX=0.0
Y_ERROR_MIN=-$Y_ERROR_MAX
Z_ERROR_MAX=0.03
Z_ERROR_MIN=-$Z_ERROR_MAX

# rotational error
ROLL_ERROR_MAX=0.1
ROLL_ERROR_MIN=-$ROLL_ERROR_MAX
PITCH_ERROR_MAX=0.1
PITCH_ERROR_MIN=-$PITCH_ERROR_MAX
YAW_ERROR_MAX=0.1
YAW_ERROR_MIN=-$YAW_ERROR_MAX

# ports
NUM_FRAMEWORKS=3
BASE_ROS_PORT=11311
BASE_GAZEBO_PORT=$(( $BASE_ROS_PORT + $NUM_EXPERIMENTS * $NUM_FRAMEWORKS )) 

EXPERIMENT_COUNTER=0

submit_job () {
    ROS_PORT=$(($BASE_ROS_PORT + $EXPERIMENT_COUNTER))
    GAZEBO_PORT=$(($BASE_GAZEBO_PORT + $EXPERIMENT_COUNTER))
    LOG_NAME=${COMMENT}_f${1}_id${2}
    echo "Submitting job with LOG_NAME $LOG_NAME, ROS_PORT $ROS_PORT, GAZEBO_PORT $GAZEBO_PORT"
    sbatch --output=$LOG_PATH/$LOG_NAME.out --error=$LOG_PATH/$LOG_NAME.err --export=FRAMEWORK=${1},SEED=${2},TIME_STEPS=${TIME_STEPS},ROS_PORT=${ROS_PORT},GAZEBO_PORT=${GAZEBO_PORT},LOG_NAME=${LOG_NAME},CLUSTER_PATH=$CLUSTER_PATH,HOME_DIR=$HOME_DIR,MAX_EP_LEN=$MAX_EP_LEN,X_ERROR_MAX=$X_ERROR_MAX,X_ERROR_MIN=$X_ERROR_MIN,Y_ERROR_MAX=$Y_ERROR_MAX,Y_ERROR_MIN=$Y_ERROR_MIN,Z_ERROR_MAX=$Z_ERROR_MAX,Z_ERROR_MIN=$Z_ERROR_MIN,ROLL_ERROR_MAX=$ROLL_ERROR_MAX,ROLL_ERROR_MIN=$ROLL_ERROR_MIN,PITCH_ERROR_MAX=$PITCH_ERROR_MAX,PITCH_ERROR_MIN=$PITCH_ERROR_MIN,YAW_ERROR_MAX=$YAW_ERROR_MAX,YAW_ERROR_MIN=$YAW_ERROR_MIN ${CLUSTER_PATH}/experiment.slurm
    EXPERIMENT_COUNTER=$(($EXPERIMENT_COUNTER + 1))
} 

for i in $(seq 1 $NUM_EXPERIMENTS); 
do 
submit_job "1" ${i} # framework 1
submit_job "2" ${i} # framework 2
submit_job "3" ${i} # framework 3
done

echo "Done."
