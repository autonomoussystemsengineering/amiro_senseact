actEmergencyStop
======

TBA


Paramter
===========

|       Parameter       |       Type       |         default value         |                                           Desciption                                           |
| --------------------- | ---------------- | ----------------------------- | ---------------------------------------------------------------------------------------------- |
| lidarinscope,         | String           | /AMiRo_Hokuyo/lidar           | Scope for receiving lidar data                                                                 |
| switchinscope         | String           | /AMiRo_Hokuyo/emergencySwitch | Scope for switching the emergency behaviour: Accepts <On,ON,on,1> or <Off,OFF,off,0> as string |
| cntMax                | Integer          | 5                             | Number of consecutive scans which need to be less than the given distance                      |
| distance              | Float            | 0.4                           | Maximum distance for emergency halt in meter                                                   |
| delay                 | Unsigned Integer | 10                            | Loop periodicity in milliseconds                                                               |
| emergencyHaltOutScope | String           | /AMiRo_Hokuyo/emergencyHalt   | Scope for publishing wether an emergency halt is performed                                     |
| doEmergencyBehaviour  | Boolean          | false                         | Enable emergency behaviour from start on                                                       | 
