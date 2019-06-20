senseRingProximity
==
This tool reads, generalizes and publishes proximity values. For each proximity sensor the original, obstacle and the edge value is published. The obstacle and edge values are based on the [[includes:sensorModels:startVCNL4020|obstacle and edge model]].

For the generalization there are loaded ground and air offsets from different files. On default the ground offsets are loaded from ''/home/root/initial/irConfig.conf'' and the air offsets from ''/home/root/initial/irEmpty.conf''. Please have also a look into the [[demo:initial:start|initial demo]].

Building
--
This is a cmake project, so just type:
  - ''cmake .''
  - ''make''

Basic Functionality
--
This tool publishes proximity values based on the [[includes:sensorModels:startVCNL4020|obstacle and edge model]]. Due to big differences in the measurement behavior of the sensors, their measured values have to be generalized for the models. In this case simple offsets are used, which have to be calculated for each sensor before using the senseRingProximity program the first time.

The sensor values are read directly from the CAN. This tool only uses RSB for publishing the generalized values.

Sensor Offsets
--
The offsets are loaded from configuration files, which are written in csv format. Using the tab seperator ("\t") the files just list the offsets from first to last sensor. There is no head line!
''<offset sensor 1>\t<offset sensor 2>\t...\t<offset sensor 8>''

There are used two types of offsets for each sensor. The first offset is the ground offset, which is used to decide, if there could be an obstacle or an edge. The second one is the air offset, which is used for the edge distance approach. On default the configuration files have to be in the directory of the [[demo:initial:start|initial demo]]:
  - ground offsets: ''/home/root/initial/irConfig.conf''
  - air offsets: ''/home/root/initial/irEmpty.conf''

The path to the ground offsets configuration file can be changed by the parameter ''-l'' (see section "Parameters") and at runtime over RSB (see section "Change Sensor Offsets at Runtime"). The air offsets configuration file cannot be changed!

The offsets are the mean values of a measurement set. For such a measurement the tool [[tools:robotTools:measureOffsetTest:start|measureOffsetTest]] can be used. It has a parameter to calculate the mean values and to write those configuration files directly. This tool has to be used twice, for each offset once, which is defined in scripts in the [[demo:initial:start|initial demo]].

For the measurement of the ground offset, the robot has to be placed on the ground, which should be used (table, floor, etc.). There shouldn't be any obstacles (including cables!) or edges in range of the sensors (for the VCNL4020 proximity sensors it's a range about 25 cm around the robot). For the measurement of the air offset, the robot should be placed in the air by using e.g. thin objects, etc. (for the VCNL4020 proximity sensors the height should be about 20 cm). Of course, there shouldn't be any obstacles in the sensors' range, either!

Change Sensor Offsets at Runtime
--
While the air offsets are fixed, the ground offsets can be changed at runtime over RSB. In this case just the path to the new configuration file, which includes the new ground offsets, has to be sent as a string over the "New Config Command" scope.

If the tool has received the path, it will response over the "Command Response" scope with the following answers:

| Response Code |                                              Description                                              |
| ------------- | ----------------------------------------------------------------------------------------------------- |
| DONE          | The path and the file are correct and the offset values are set.                                      |
| ERROR         | There was a problem with the path or the file. The offsets couldn't be set. The old offsets are used. |

Published Sensor Values
--
There are published two values: the obstacle and the edge value. Given the sensor value s, the obstacle (o) and edge values (e) are calculated with the ground (g) and air offsets (a) as follows:
  * ''if (s >= g)''
    * ''o = s - g''
    * ''e = g - a''
  * ''else''
    * ''o = 0''
    * ''if (s > a)''
      * ''e = s - a''
    * ''else''
      * ''e = 0''
  * ''e = e / (g - a) * 10000''

Important is the difference between these two sensor values: The obstacle value as the real distance to the ground offset in sensor ticks. The edge value is a relative value between the ground and the air offset. The "measured" value in range [air offset, ground offset] will be normalized to [0,1]. Due to integer communication it is ranged up by 10000 (saving the next 4 decimal places).

On default, the ground and edge values are both calculated and published seperately via RSB. They can be used directly for the obstacle and the edge model without any additional generalization. By using application arguments (see section "Parameters"), the calculation and publication of the ground and the edge values can be turned off. The original values are always published.

RSB Scopes
--
|        Name        |        Default Scope        |                                       Description                                        |
| ------------------ | --------------------------- | ---------------------------------------------------------------------------------------- |
| Original Values    | /rir_prox/original          | Scope for sending the original proximity sensor values.                                  |
| Obstacle Values    | /rir_prox/obstacle          | Scope for sending the generalized proximity sensor values for the obstacle model.        |
| Edge Values        | /rir_prox/ground            | Scope for sending the generalized proximity sensor values for the edge model.            |
| New Config Command | /rir_prox/newconfig         | Scope for receiving the path (String) of the new configuration file with ground offsets. |
| Command Response   | /rir_prox/newconfigResponse | Scope for sending the command response.                                                  |

Parameters
--
|       Parameter        |  Type   |           Default Value            |                              Description                               |
| ---------------------- | ------- | ---------------------------------- | ---------------------------------------------------------------------- |
| --outscopeOriginal, -o | String  | "/rir_prox/original"               | Scope for sending original proximity values.                           |
| --outscopeObstacle, -e | String  | "/rir_prox/obstacle"               | Scope for sending generalized proximity values for the obstacle model. |
| --outscopeGround, -g   | String  | "/rir_prox/ground"                 | Scope for sending generalized proximity values for the edge model.     |
| --inscopeCommand, -i   | String  | "/rir_prox/newconfig"              | Scope for receiving new configuration path command.                    |
| --outscopeResponse, -r | String  | "/rir_prox/newconfigResponse"      | Scope for sending command response.                                    |
| --period, -t           | Integer | 0                                  | Update interval in milliseconds (0 for maximum rate).                  |
| --print, -p            | -       | -                                  | Prints read proximity values in the console.                           |
| --loadOffsetFile, -l   | String  | "/home/root/initial/irConfig.conf" | File name for the obstacle offsets.                                    |
| --noObstacleValues     | -       | -                                  | Flag, if the obstacle values for the obstacle model shouldn't be sent. |
| --noEdgeValues         | -       | -                                  | Flag, if the edge values for the edge model shouldn't be sent.         |
