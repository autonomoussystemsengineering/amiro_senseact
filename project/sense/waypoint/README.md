waypoint
============

After initialization a reference laser scan is taken.
This program detects whether the current laser scan differs from this reference laser scan and
publishes this via rsb.

RSB Scopes
==========

| Scope (default name) |                                Description                                |
| -------------------- | ------------------------------------------------------------------------- |
| /AMiRo_Hokuyo/lidar  | Inscope for the lidar data                                                |
| /waypoint/command    | Inscope for commands to initialize ("init") or stop ("stop") the waypoint |
| /waypoint/state      | Outscope for the current state ("entered" or "left" will be send)         |

Important Command-Line Options
==========
|     Option      |                               Description                                |
| --------------- | ------------------------------------------------------------------------ |
| -h, --help      | Display a help message                                                   |
| -s, --startNow  | Initializes the waypoint immediately without waiting for an init command |
| -r, --range     | Range of detection in m (default: 1m)                                    |
| --diffThreshold | Theshold in m; Changes below the threshold are ignored (default: 0.3m)   |
