Act Amiro Vector
==

This tool sets target movement vector for the AMiRo.
The maximum absolute lenght of the vector is 1, without any units (the realisation is done by the AMiRo basic boards).

Building
--

This is a cmake project, so just type:
  - ''cmake .''
  - ''make''

RSB Scopes
--

For communication there are RSB scopes defined.

|        Type           | Default scope |         Description          |
| --------------------- | ------------- | ---------------------------- |
| rst::math::Vec2DFloat | "/vector"     | Inscope for vector commands. |
