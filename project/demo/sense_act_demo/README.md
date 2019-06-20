# Sense Act Demo

This is the demo project for the AMiRo which builds all binaries and other necessary files like 'rsb.conf' and 'start/stop scripts to one folder that can be copied to the AMiRo.

## Usage
Just run the "install.sh" script which runs the cmake part and then all binaries and 'rsb.conf', 'start.sh' and 'stop.sh' to the 'ros_sense_act_tools' folder.
Just copy this folder with the following command to the AMiRo:
```
ssh -r ros_sense_act_tools root@<AMiRo-IP>:<Destination>
```
