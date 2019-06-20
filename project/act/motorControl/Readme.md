Motor Control
============

Receives motor commands with priority and duration. Manages them and sets the wheel velocities accordingly via CAN. Commands (behaviors) with lower priority number (= higher priority) override commands with higher priority number (= lower priority). If a command expires, the execution of the next highest prioritised command starts.
Example:
	- A wandering behavior may have priority 3.
	- An edge avoidance behavior should then have priority 2, in order to prevent the table top robot from falling from the table.
	- Finally, a keyboard controlled behavior should have priority 1, so all behaviors can always be overwritten by manual keyboard steering.

Building
==========

This is a cmake project, so just type:
  - ''cmake .''
  - ''make''

RSB Scopes
==========

For communication there are RSB scopes defined.

|    Name     |        Type         |    Default scope    |         Description         |
| ----------- | ------------------- | ------------------- | --------------------------- |
| Motor scope | rst::generic::Value | "/motor/[priority]" | Inscope for motor commands. |

How to give a command
==========

Publish a command on the motor scope as a vector of 3 integers (boost::shared_ptr<rst::generic::Value >) with forward velocity (µm/ms), angular velocity (µrad/ms) and duration (µs) in that order.
Special command: If the duration is 0 the command will be executed only once and expire immediately (e.g. stop command).

Parameters
==========

For parameter information type ''motorControl -h''.

|  Parameter  |  Type  | Default Value |            Description            |
| ----------- | ------ | ------------- | --------------------------------- |
| --inscope,i | String | "/motor"      | Scope for receiving the commands. |
