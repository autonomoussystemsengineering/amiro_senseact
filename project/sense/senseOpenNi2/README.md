senseOpenNI2
============

This is an OpenNI2 driver which greps the a depth camera device over the OpenNI2 interface and publishes the frames (CV_8UC1) via rsb.

|  Parameter   |  Type   | default value |                                 Desciption                                  |
| ------------ | ------- | ------------- | --------------------------------------------------------------------------- |
| outscope, o  | String  | /depthImage   | Scope for sending images                                                    |
| depthMode    | Integer | -             | Mode of Depth Image.                                                        |
| printInfo, p | -       | -             | Flag if just the camera infos shall be printed. The tool closes afterwards. |
