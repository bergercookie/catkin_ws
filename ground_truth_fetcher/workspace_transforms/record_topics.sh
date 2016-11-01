#!/usr/bin/env bash

rosbag record -O records.bag /ar_multi_boards_top_right/transform /ar_multi_boards_top_left/transform --duration 15
rostopic echo -b records.bag -p /ar_multi_boards_top_right/transform > transform_right_to_origin.txt
rostopic echo -b records.bag -p /ar_multi_boards_top_left/transform > transform_left_to_origin.txt
