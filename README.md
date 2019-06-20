Sense Act Plan/ROS project
====

How To Setup Repo
==
* clone all submodules ```git submodule update --init --recursive```
* source enviroments (this has do be done for every working terminal) ```source source.sh``` 
* create protobuff files ```cd project/includes/types && chmod +x createPbFiles.sh && ./createPbFiles.sh <rsb.version> && cd ../../../```
* copy rsb.conf to ROS-home ```cp catkin_ws/rsb.conf ~/.ros/```
* to build ROS workspace: ```cd catkin_ws && source devel/setup.bash && catkin_make -DCMAKE_BUILD_TYPE=Release``` if there is no devel folder then ```source /opt/ros/kinetic/setup.bash``` instead
