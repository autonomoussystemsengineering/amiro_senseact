senseHokuyo
============

This is a Hokuyo rsb informer.


|   Parameter    |  Type   | default value  |                                                                                 Desciption                                                                                  |
| -------------- | ------- | -------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| outscope, o    | String  | /depthImage    | Scope for sending scans                                                                                                                                                     | 
| device, d      | string  | /dev/ttyACM0   | Device name (e.g. /dev/ttyACM0)                                                                                                                                             |
| publishNthScan | size_t  | 1              | Publish only every n'th scan via RSB                                                                                                                                        |
| baudrate, b    | Integer | 115200         | Communication baud rate (does not matter for USB connection)                                                                                                                |
| scanName, n    | string  | range          | [range , top_urg_range+intensity , range+intensity1+AGC1]                                                                                                                   |
| scanStart, s   | Integer | 44             | Index where to start of the scan (e.g. Min 44)                                                                                                                              |
| scanEnd, e     | Integer | 725            | Index where to end of the scan (e.g. Max 725)                                                                                                                               |
| test           | Boolean | false          | Performing test without any RSB interaction                                                                                                                                 |
| encoding       | Integer | HOKUYO_3DIGITS | [2 , 3] 2 or 3 char encoding. 04LX supports both, but 30LX only 3-char 2-char encoding reduces range to 4meters, but improves data transfer rates over standard serial port |
