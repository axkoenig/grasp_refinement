#!/bin/bash

log_name="first_experiments.csv"

echo "Starting simulation in headless mode."
gnome-terminal -- roslaunch description reflex.launch run_teleop_node:=false gui:=true

echo "Sleeping until simulation is launched."
sleep 5s

# ensures that we exit shell script upon Ctrl+c
trap "exit" INT

for polar in 0 0.25 0.5 0.75 1.0 1.25 1.5
do  
    echo "Starting baseline controller in new terminal"
    roslaunch brain baseline_commander.launch log_name:=$log_name polar:=$polar
done