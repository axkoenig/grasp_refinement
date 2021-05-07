#!/bin/bash

# transl offset
x_error_max=0.05
x_error_min=-$x_error_max
y_error_max=0.02
y_error_min=-$y_error_max
z_error_max=0.05
z_error_min=-$z_error_max

# rot offset
roll_error_max=0.1
roll_error_min=-$roll_error_max
pitch_error_max=0.1
pitch_error_min=-$pitch_error_max
yaw_error_max=0.1
yaw_error_min=-$yaw_error_max

time_steps=3000
rosclean purge -y

read -r -d '' HPARAMS << EOM
--time_steps=$time_steps \ 
--max_ep_len=15 \
--x_error_min ${x_error_min} --x_error_max ${x_error_max} \
--y_error_min ${y_error_min} --y_error_max ${y_error_max} \
--z_error_min ${z_error_min} --z_error_max ${z_error_max} \
--roll_error_min ${roll_error_min} --roll_error_max ${roll_error_max} \
--pitch_error_min ${pitch_error_min} --pitch_error_max ${pitch_error_max} \
--yaw_error_min ${yaw_error_min} --yaw_error_max ${yaw_error_max}
EOM

trap "exit" INT
for i in 1 2 3 4 5 6 7 8 9 10; do
    python main.py --framework=1 --seed=${i} --log_name=04May_Rot_MultObj_f1_id${i} $HPARAMS
    python main.py --framework=2 --seed=${i} --log_name=04May_Rot_MultObj_f2_id${i} $HPARAMS
    python main.py --framework=3 --seed=${i} --log_name=04May_Rot_MultObj_f3_id${i} $HPARAMS
done

echo "Done! Have a nice day."
