#!/bin/bash
num_exp=20

trap "exit" INT
for i in 0 0.01 0.05 0.1 0.15 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 1
do
    python main.py --log_name=wed1703_weight${i} --time_steps=10000 --reward_weight=${i}
done

echo "Done! Have a nice day."