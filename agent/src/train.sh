#!/bin/bash

rosclean purge -y

error_low=-0.04
error_high=-0.04

trap "exit" INT
for i in 0 1 2 3 4 5 6
do
    python main.py --framework=1 --log_name=19Apr_Night_Framework1_X0.03_try${i} --time_steps=10000 --max_ep_len=15 --x_error ${error_low} ${error_high} --y_error ${error_low} ${error_high} --z_error ${error_low} ${error_high} 
    python main.py --framework=2 --log_name=19Apr_Night_Framework2_X0.03_try${i} --time_steps=10000 --max_ep_len=15 --x_error ${error_low} ${error_high} --y_error ${error_low} ${error_high} --z_error ${error_low} ${error_high} 
done

echo "Done! Have a nice day."