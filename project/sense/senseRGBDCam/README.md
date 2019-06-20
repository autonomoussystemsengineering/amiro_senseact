sense RGB Depth Camera
====

This tool is for reading images from depth cameras.

Functionality
====

By using the library OpenNI the RGB or IR image and the depth image will be loaded from the depth camera and directly converted to OpenCV. Afterwards the images will be published via RSB. For examples of the usage of this program, please have a look into the follwoing scripts:

|     Script      |                                Description                                 |
| --------------- | -------------------------------------------------------------------------- |
| readRGB.sh      | Starts sending the RGB and depth image.                                    |
| readIR.sh       | Starts sending the IR and depth image.                                     |
| getParamsRGB.sh | Prints the camera parameters and options for the RGB and the depth stream. |
| getParamsIR.sh  | Prints the camera parameters and options for the IR and the depth stream.  |

All four scripts are sending a fused image of the RGB or IR image and the depth image via RSB over the RGB Scope.

Scopes
====

| Scope Name  |     Scope     |                                                                   Description                                                                   |
| ----------- | ------------- | ----------------------------------------------------------------------------------------------------------------------------------------------- |
| RGB Scope   | /images/rgb   | Scope for sending the RGB or the IR image. On debug mode this scope is also used to send both RGB or IR and the depth image fused in one image. |
| Depth Scope | /images/depth | Scope for sending the depth image.                                                                                                              |

Parameters
====

|    Parameter     |  Type   | Default Value |                                     Description                                      |
| ---------------- | ------- | ------------- | ------------------------------------------------------------------------------------ |
| RGBOutscope, r   | String  | /images/rgb   | Scope for sending RGB or IR images.                                                  |
| DepthOutscope, d | String  | /images/depth | Scope for sending depth images.                                                      |
| printTime, t     | -       | -             | Prints the process time of single processes.                                         |
| rgbMode          | Integer | -             | Mode of RGB Image.                                                                   |
| depthMode        | Integer | -             | Mode of Depth Image.                                                                 |
| irMode           | Integer | -             | Mode of IR Image.                                                                    |
| period           | Integer | 10            | Period between two image fetches in ms.                                              |
| compression, c   | Integer | 100           | Compression value of the image in range [0,100].                                     |
| showIR           | -       | -             | Flag if the IR image shall be shown instead of the RGB Image.                        |
| sendImage, s     | -       | -             | Flag if the image shall be converted to OpenCV and send via RSB.                     |
| printInfo, p     | -       | -             | Flag if just the camera infos shall be printed. The tool closes afterwards.          |
| debugImage, i    | -       | -             | Flag if over the RGB scope the RGB image and depth image shall be send in one image. |
