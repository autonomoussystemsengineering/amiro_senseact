Act Amiro Motor
==

This tool sets the speed of the two AMiRo wheels. It requires expects a rst::generic::Value of type ARRAY with two INT entries. The first entry specifies the forward velocity in µm/s and the second entry specifies the rotation speed in µrad/s.

Building
--

This is a cmake project, so just type:
  - ''cmake .''
  - ''make''

RSB Scopes
--

For communication there are RSB scopes defined.

|        Type         | Default scope |         Description         |
| ------------------- | ------------- | --------------------------- |
| rst::generic::Value | "/motor"      | Inscope for motor commands. |
