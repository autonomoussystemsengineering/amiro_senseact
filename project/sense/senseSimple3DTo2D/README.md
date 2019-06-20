senseSimple3DTo2D
===========

This is a simple laser scanner immitation for a RGBD camera.
The resolution and other configurations need to be set via ''/usr/lib/OpenNI2/Drivers/orbbec.ini''.

Paramter
===========

|      Parameter       |  Type  | default value |                                         Desciption                                          |
| -------------------- | ------ | ------------- | ------------------------------------------------------------------------------------------- |
| outscope, o          | string | /lidar        | Scope for sending lidar data                                                                |
| lidarPublishDelay, d | size_t | 1000          | Period for publishing laser data in ms                                                      |
| justCenter, c        | Flag   |               | Flag, if not the minimum, but just the vertical centered value shall be used as laser scan. | 
