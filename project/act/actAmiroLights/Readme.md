Act Amiro Lights
===
This tool can set the color and the brightness of all lights of the AMiRo lightring at once. Each light of the AMiRo lightring requires three values between 0 and 255 for red green and blue. An extra value between 0 and 100 is required to set percentual brightness of all lights.

Building
--

This is a cmake project, so just type:
  - ''cmake .''
  - ''make''

Communication
--

This tool listens on the scope /lights and expects a rst::generic::Value of type rst::generic::Value::ARRAY with 25 entries of type rst::generic::Value::INT.

| Default Scope |     Scope Type      |             Description             |
| ------------- | ------------------- | ----------------------------------- |
| /lights       | rst::generic::Value | Scope for receiving light commands. |


Example
--

The following table contains four examples of vectors to set the AMiRo lightring lights.

| Index | rainbow | red | green | blue |
| ----- | ------- | --- | ----- | ---- |
| 01    | 255     | 255 | 0     | 0    |
| 02    | 0       | 0   | 255   | 0    |
| 03    | 0       | 0   | 0     | 255  |
| 04    | 255     | 223 | 0     | 0    |
| 05    | 127     | 0   | 223   | 0    |
| 06    | 0       | 0   | 0     | 223  |
| 07    | 127     | 191 | 0     | 0    |
| 08    | 127     | 0   | 191   | 0    |
| 09    | 0       | 0   | 0     | 191  |
| 10    | 127     | 159 | 0     | 0    |
| 11    | 255     | 0   | 159   | 0    |
| 12    | 0       | 0   | 0     | 159  |
| 13    | 0       | 127 | 0     | 0    |
| 14    | 255     | 0   | 127   | 0    |
| 15    | 0       | 0   | 0     | 127  |
| 16    | 0       | 95  | 0     | 0    |
| 17    | 127     | 0   | 95    | 0    |
| 18    | 127     | 0   | 0     | 95   |
| 19    | 0       | 63  | 0     | 0    |
| 20    | 0       | 0   | 63    | 0    |
| 21    | 255     | 0   | 0     | 63   |
| 22    | 127     | 31  | 0     | 0    |
| 23    | 0       | 0   | 31    | 0    |
| 24    | 127     | 0   | 0     | 31   |
| 25    | 50      | 100 | 100   | 100  |
