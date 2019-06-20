senseCam
============

This is a camera driver which greps the given camera device over the OpenCV interface.
Every single frame is published via a given scope.

Paramter
===========

|   Parameter    |  Type   | default value |                     Desciption                     |
| -------------- | ------- | ------------- | -------------------------------------------------- |
| outscope, o    | String  | /image        | Scope for sending images via RSB                   |
| device,d       | Integer | 0             | Number of device /dev/video#                       |
| compression, c | Integer | 0             | Enable image compression with value betweeen 1-100 |
| flip, f        | Flag    |               | Flag to flip the output image around x and y axis  | 
