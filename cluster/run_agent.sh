#!/bin/bash

# parse arguments
while [[ "$#" -gt 0 ]]
do
case $1 in
    -x|--x_error)
    X_ERROR=$2
    ;;
    -y|--y_error)
    Y_ERROR=$2
    ;;
    -z|--z_error)
    Z_ERROR=$2
    ;;
    -t|--time_steps)
    TIME_STEPS=$2
    ;;
    -f|--framework)
    FRAMEWORK=$2
    ;;
    -l|--max_ep_len)
    MAX_EP_LEN=$2
    ;;
    -n|--log_name)
    LOG_NAME=$2
    ;;
esac
shift
done

echo "Running agent with the following args:"
echo "X_ERROR: $X_ERROR"
echo "Y_ERROR: $Y_ERROR"
echo "Z_ERROR: $Z_ERROR"
echo "TIME_STEPS: $TIME_STEPS"
echo "FRAMEWORK: $FRAMEWORK"
echo "MAX_EP_LEN: $MAX_EP_LEN"
echo "LOG_NAME: $LOG_NAME"

. /opt/ros/noetic/setup.sh
. ~/overlay/work/catkin_ws/devel/setup.sh
cd ~/overlay/work/catkin_ws/src/grasp_refinement/agent/src

echo "Entering training loop..."
python3 main.py --framework=${FRAMEWORK} --output_dir=/output/agents --log_name=${LOG_NAME} --time_steps=${TIME_STEPS} --max_ep_len=${MAX_EP_LEN} --x_error -${X_ERROR} ${X_ERROR} --y_error -${Y_ERROR} ${Y_ERROR} --z_error -${Z_ERROR} ${Z_ERROR}             
echo "Done! Have a nice day."
