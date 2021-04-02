#!/bin/bash

trap "exit" INT
for i in 1 2 3 4 5 6
do
    python main.py --log_name=26Mar_PosInputOnlyWhenContact_wErrorsX0.03_try${i} --time_steps=10000 --max_ep_len=15 --x_error=0.03
done

echo "Done! Have a nice day."