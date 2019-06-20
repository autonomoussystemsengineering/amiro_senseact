#!/bin/bash
./CoreSLAMBasic --odominscope /odom --lidarinscope /lidar --senImage 1 --delta 0.01 --sigma_xy 0.01 &
wait
