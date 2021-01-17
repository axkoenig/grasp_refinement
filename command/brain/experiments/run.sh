#!/bin/bash
log_name="closeBeforeResetting_higherWaypoint.csv"

# experiment parameters 
num_steps=25
polar_start=-1.5
polar_end=1.5
azimuthal_start=0
azimuthal_end=3

# calculations for intermediate steps
polar_range=$(bc <<< "scale=4;($polar_end)-($polar_start)")
azimuthal_range=$(bc <<< "scale=4;($azimuthal_end)-($azimuthal_start)")
polar_step=$(bc <<< "scale=4;($polar_range)/($num_steps)")
azimuthal_step=$(bc <<< "scale=4;($azimuthal_range)/($num_steps)")

# estimations about 
num_exp=$(bc <<< "scale=4;($num_steps+1)^2")
exp_duration=30
all_exp_duration=$(bc <<< "scale=4;$num_exp*$exp_duration")
date=$(date --date="+$all_exp_duration seconds" '+%Y-%m-%d %T')

GREEN='\033[0;32m'
NC='\033[0m'
echo "You selected $num_steps intermediate steps."
echo "Final number of experiments will be $num_exp."
echo -e "${GREEN}Estimated finish: $date ${NC}"

# wait for confirmation (user checks if estimated finish is fine)
read -p "Press enter to continue."

echo "Starting simulation in headless mode."
gnome-terminal -- roslaunch description reflex.launch gui:=false

sleep=10
echo "Sleeping for $sleep seconds until simulation is launched."
sleep ${sleep}s

# setup sequence over which we iterate
polar_vals=($(seq $polar_start $polar_step $polar_end))
azimuthal_vals=($(seq $azimuthal_start $azimuthal_step $azimuthal_end))

# ensures that we exit shell script upon Ctrl+c
trap "exit" INT
for polar in ${polar_vals[@]}
do  
    for azimuthal in ${azimuthal_vals[@]}
    do
        roslaunch brain baseline_commander.launch log_name:=$log_name polar:=$polar azimuthal:=$azimuthal
    done 
done

echo "Done! Have a nice day."