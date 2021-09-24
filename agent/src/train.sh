#!/bin/bash

# transl offset
x_error_max=0.05
x_error_min=-$x_error_max
y_error_max=0.05
y_error_min=-$y_error_max
z_error_max=0.05
z_error_min=-$z_error_max

# rot offset
roll_error_max=10
roll_error_min=-$roll_error_max
pitch_error_max=10
pitch_error_min=-$pitch_error_max
yaw_error_max=10
yaw_error_min=-$yaw_error_max

# reward weights
w_eps_torque=1
w_delta=1

time_steps=2000
prefix="test"

# occasionally run this
rosclean purge -y

read -r -d '' HPARAMS <<EOM
--time_steps=$time_steps \ 
--max_ep_len=15 \ 
--w_eps_torque ${w_eps_torque} \ 
--w_delta ${w_delta} \
--x_error_min ${x_error_min} --x_error_max ${x_error_max} \
--y_error_min ${y_error_min} --y_error_max ${y_error_max} \
--z_error_min ${z_error_min} --z_error_max ${z_error_max} \
--roll_error_min ${roll_error_min} --roll_error_max ${roll_error_max} \
--pitch_error_min ${pitch_error_min} --pitch_error_max ${pitch_error_max} \
--yaw_error_min ${yaw_error_min} --yaw_error_max ${yaw_error_max}
EOM

# evaluate model
# python main.py --reward_framework=1 --train=0 $HPARAMS --test_model_path=/home/parallels/cluster_logs/final_model.zip

trap "exit" INT
for i in 1 2 3 4 5 6 7 8 9 10; do
    python main.py --reward_framework=1 --seed=${i} --log_name=${prefix}_f1_id${i} $HPARAMS
    python main.py --reward_framework=2 --seed=${i} --log_name=${prefix}_f2_id${i} $HPARAMS
    python main.py --reward_framework=3 --seed=${i} --log_name=${prefix}_f3_id${i} $HPARAMS
done

echo "Done! Have a nice day."


