#!/bin/bash

#### USAGE
# ./experiments.sh -n 10 -c 05May_Test
####

# parse arguments
while [[ "$#" -gt 0 ]]; do
    case $1 in
    -n | --num_experiments)
        NUM_EXPERIMENTS=$2
        ;;
    esac
    case $2 in
    -c | --comment)
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
TIME_STEPS=10000

# translational error
X_ERROR_MAX=0.05
X_ERROR_MIN=-$X_ERROR_MAX
Y_ERROR_MAX=0.05
Y_ERROR_MIN=-$Y_ERROR_MAX
Z_ERROR_MAX=0.05
Z_ERROR_MIN=-$Z_ERROR_MAX

# rotational error
ROLL_ERROR_MAX=10
ROLL_ERROR_MIN=-$ROLL_ERROR_MAX
PITCH_ERROR_MAX=10
PITCH_ERROR_MIN=-$PITCH_ERROR_MAX
YAW_ERROR_MAX=10
YAW_ERROR_MIN=-$YAW_ERROR_MAX

# ports
NUM_FRAMEWORKS=8
BASE_ROS_PORT=11711
BASE_GAZEBO_PORT=$(($BASE_ROS_PORT + $NUM_EXPERIMENTS * $NUM_FRAMEWORKS))

EXPERIMENT_COUNTER=0

submit_job() {
    ROS_PORT=$(($BASE_ROS_PORT + $EXPERIMENT_COUNTER))
    GAZEBO_PORT=$(($BASE_GAZEBO_PORT + $EXPERIMENT_COUNTER))
    LOG_NAME=${COMMENT}_f${1}_s${4}_id${2}_algo${3}_e${5}_d${6}
    echo "Submitting job with LOG_NAME $LOG_NAME, ROS_PORT $ROS_PORT, GAZEBO_PORT $GAZEBO_PORT"
    sbatch --output=$LOG_PATH/$LOG_NAME.out --error=$LOG_PATH/$LOG_NAME.err --export=FRAMEWORK=${1},SEED=${2},ALGORITHM=${3},FORCE_SENSING=${4},TIME_STEPS=${TIME_STEPS},W_EPS_TORQUE=${5},W_DELTA=${6},ROS_PORT=${ROS_PORT},GAZEBO_PORT=${GAZEBO_PORT},LOG_NAME=${LOG_NAME},CLUSTER_PATH=$CLUSTER_PATH,HOME_DIR=$HOME_DIR,X_ERROR_MAX=$X_ERROR_MAX,X_ERROR_MIN=$X_ERROR_MIN,Y_ERROR_MAX=$Y_ERROR_MAX,Y_ERROR_MIN=$Y_ERROR_MIN,Z_ERROR_MAX=$Z_ERROR_MAX,Z_ERROR_MIN=$Z_ERROR_MIN,ROLL_ERROR_MAX=$ROLL_ERROR_MAX,ROLL_ERROR_MIN=$ROLL_ERROR_MIN,PITCH_ERROR_MAX=$PITCH_ERROR_MAX,PITCH_ERROR_MIN=$PITCH_ERROR_MIN,YAW_ERROR_MAX=$YAW_ERROR_MAX,YAW_ERROR_MIN=$YAW_ERROR_MIN ${CLUSTER_PATH}/experiment.slurm
    EXPERIMENT_COUNTER=$(($EXPERIMENT_COUNTER + 1))
}

for i in $(seq 1 $NUM_EXPERIMENTS); do
    # params: reward_framewok, seed, algorithm, force_framework, W_EPS_TORQUE, W_DELTA
    submit_job "1" ${i} "sac" "1" "10" "1"
    submit_job "1" ${i} "sac" "1" "8" "1"
    submit_job "1" ${i} "sac" "1" "5" "1"
    submit_job "1" ${i} "sac" "1" "1" "1"
    
    submit_job "1" ${i} "sac" "1" "8" "0.2"
    submit_job "1" ${i} "sac" "1" "8" "0.7"
    submit_job "1" ${i} "sac" "1" "8" "1.3"
    submit_job "1" ${i} "sac" "1" "8" "2"
done

echo "Done."
