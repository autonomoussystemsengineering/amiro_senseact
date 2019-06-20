#!/bin/bash
./stopWaypoint.sh
# Start the spread communication deamon
spread  &
sleep 5
./senseHokuyo -o /scan &
./waypoint -s --lidarinscope /scan &
sleep 3
#rsb-sendcpp0.11 /waypoint/command init.txt
echo "Init waypoint"
