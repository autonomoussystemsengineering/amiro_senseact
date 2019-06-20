#!/bin/bash
killall showCamJpg
./showCamJpg -i /AMiRo_Hokuyo_Pitch_9/ogmEdge &
./showCamJpg -i /AMiRo_Hokuyo_Pitch_9/ogmWeed &
./showCamJpg -i /AMiRo_Hokuyo_Pitch_9/ogmCrop &
./showCamJpg -i /AMiRo_Hokuyo_Pitch_9/ogmFloor &
