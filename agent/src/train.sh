#!/bin/bash

x_error_max=0.03
x_error_min=-$x_error_max
y_error_max=0
y_error_min=-$y_error_max
z_error_max=0.02
z_error_min=-$z_error_max
time_steps=3000

rosclean purge -y

trap "exit" INT
for i in 0 1 2 3 4 5 6 7 8 9 10 11 12 13
do
    # python main.py --framework=1 --log_name=30Apr_F1_Lim3_X0.03_Z0.02_try${i} --time_steps=$time_steps --max_ep_len=15 --x_error_min ${x_error_min} --x_error_max ${x_error_max} --y_error_min ${y_error_min} --y_error_max ${y_error_max} --z_error_min ${z_error_min} --z_error_max ${z_error_max} 
    # python main.py --framework=2 --log_name=30Apr_F2_Lim3_X0.03_Z0.02_try${i} --time_steps=$time_steps --max_ep_len=15 --x_error_min ${x_error_min} --x_error_max ${x_error_max} --y_error_min ${y_error_min} --y_error_max ${y_error_max} --z_error_min ${z_error_min} --z_error_max ${z_error_max} 
    python main.py --framework=3 --log_name=03May_F3_Lim3_X0.03_Z0.02_RW1_try${i} --time_steps=$time_steps --max_ep_len=15 --x_error_min ${x_error_min} --x_error_max ${x_error_max} --y_error_min ${y_error_min} --y_error_max ${y_error_max} --z_error_min ${z_error_min} --z_error_max ${z_error_max} 
done

echo "Done! Have a nice day." 