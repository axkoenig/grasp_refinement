#!/bin/bash

log_name="morning_fixed.csv"

echo "Starting simulation in headless mode."
gnome-terminal -- roslaunch description reflex.launch run_teleop_node:=false gui:=false

echo "Sleeping until simulation is launched."
sleep 10s

# ensures that we exit shell script upon Ctrl+c
trap "exit" INT

for polar in 0 0.25 0.5 0.75 1.0 1.25 1.5 -0.25 -0.5 -0.75 -1.0 -1.25 -1.5
do  
    for azimuthal in 0 0.25 0.5 0.75 1.0 1.25 1.5 1.75 2 2.25 2.5 2.75 3
    do
        echo "Starting baseline controller in new terminal"
        roslaunch brain baseline_commander.launch log_name:=$log_name polar:=$polar azimuthal:=$azimuthal
    done 
done

# for polar in -1.5 -1.4 -1.3 -1.2 -1.1 -1.0 -0.9 -0.8 -0.7 -0.6 -0.5 -0.4 -0.3 -0.2 -0.1 0 1.5 1.4 1.3 1.2 1.1 1.0 0.9 0.8 0.7 0.6 0.5 0.4 0.3 0.2 0.1
# do  
#     for azimuthal in 3.0 2.9 2.8 2.7 2.6 2.5 2.4 2.3 2.2 2.1 2.0 1.9 1.8 1.7 1.6 1.5 1.4 1.3 1.2 1.1 1.0 0.9 0.8 0.7 0.6 0.5 0.4 0.3 0.2 0.1
#     do
#         echo "Starting baseline controller in new terminal"
#         roslaunch brain baseline_commander.launch log_name:=$log_name polar:=$polar azimuthal:=$azimuthal
#     done 
# done

echo "Done! Have a nice day."