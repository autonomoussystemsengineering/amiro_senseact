Set Lights
============

This tool can set the lights in different colors and in different lighting types. Please use the lighting types of the [[includes:actModels:startLightModel|Light Model]].

Building
==========

This is a cmake project, so just type:
  - ''cmake .''
  - ''make''

Communication
==========

This tool just listens to a rst::generic::Value of type ARRAY. It contains the lighting type, the specified colors and finally the period time. The lighting type is always given at the first position, the period time at the last position. The colors are set in between with their RGB channels in the order red, green, blue. The color count depends on the lighting type.

Vector example with only one color:

| Position |          Content           |
| -------- | -------------------------- |
| 01       | Lighting Type              |
| 02       | Red channel of the color   |
| 03       | Green channel of the color |
| 04       | Blue channel of the color  |
| 05       | Period Time                |

Lighting Type and Color Count
========

The different lighting types are descriped in the [[includes:actModels:startLightModel|Light Model]]. For the color count in the command vector there are basically two important characteristics of the lighting types: If the lighting type uses initial colors and if the lighting type only uses one color per LED (prefix SINGLE) or if it is changing the colors (prefix CHANGE).

If the lighting type uses the initial colors, it is enough, if the the command vector only contains the lighting type and the period time (command vector length = 2). In case of the lighting type SINGLE_INIT, the initial colors will be set for each LED, otherwise, if the lighting type is CHANGE_INIT, all LEDs will set to the same color, but the color will change as defined in the initial color set.

But if the lighting type is not using initial colors, then the colors have to be given:
If the lighting type uses only one color (prefix SINGLE), then there has to be given only one color for all LEDs (command vector length = 5) or every LED gets its own color, so the command vector has to contain 8 colors (command vector length = 26). If the lighting type changes the color (prefix CHANGE), then there has to be given a list of colors with at least one color (command vector length = 2+c*3 with c > 0). The LEDs' colors will be changed (in a rhythm specified by the lighting type) in the order of the given color list, while all LEDs get the same color at once.

Period Time
========

The period time has to be given in 100 ms steps. If it is not given in these steps, it will be reased up to the next 100 ms step. Additionally every lighting type has its own minimal period time, which is defined in the [[includes:actModels:startLightModel|Light Model]]. If the period time is too small, it will be reased up to the minimal time of the lighting type.

Building the Command Vector
========

For building the command vector, the functions in the [[includes:actModels:startLightModel|Light Model]] can be used. They also return an error value, if there are wrong configurations (color count, etc.).

Example
==========

An example of how to use the [[includes:actModels:startLightModel|Light Model]] and how to send the light command can be found in the [[tools:examples:sendLights:start|sendLights tool]]. The RSB communication and the usage of the [[includes:actModels:startLightModel|Light Model]] is implemented. Please also recognize the ''CMakeLists.txt''.

RSB Scopes
==========

|  Name   | Default Scope |     Scope Type      |             Description             |
| ------- | ------------- | ------------------- | ----------------------------------- |
| Command | /amiro/lights | rst::generic::Value | Scope for receiving light commands. |

Parameters
==========

|    Parameter    |  Type  | Default Value |         Description          |
| --------------- | ------ | ------------- | ---------------------------- |
| commandScope, c | String | /amiro/lights | Inscope for commands.        |
| host            | String | 127.0.0.1     | Host for Programatik Spread. |
| port            | String | 4803          | Port for Programatik Spread. |
