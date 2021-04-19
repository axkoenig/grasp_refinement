#!/bin/bash

trap "exit" INT
for i in 0 1 2 3 4 5 6
do
    python main.py --framework=1 --log_name=18Apr_Night_Framework1_X0.03_try${i} --time_steps=10000 --max_ep_len=15 --x_error=0.03
    python main.py --framework=2 --log_name=18Apr_Night_Framework2_X0.03_try${i} --time_steps=10000 --max_ep_len=15 --x_error=0.03
done

echo "Done! Have a nice day."