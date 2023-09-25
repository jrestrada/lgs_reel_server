# lgs_reel_server
ROS2 Python node containing an action server that activates the Lateral Gamma Scanner's motorized reel and its tether-guiding carriage.

Run using:
```console
$ ros2 run lgs_reel_server reel_server_node.py
```

This server expects a custom action defined by the [lgs_interfaces package](https://github.com/jrestrada/lgs_interfaces/)

This action can be requested from a terminal using the following command:

```console
$ ros2 action send_goal --feedback turn_reel lgs_interfaces/action/Reel '{command: {velocity: 1, interval: '2.0', continuous: True}}'
```
When an action request is accepted, the node publishes the *velocity* as a Twist message through the "/cmd_vel" topic,
 to which the roboclaw driver node "/reel_motor_driver" is subscribed. This publishing will happen at intervals
determined by the field *interval*, and either a single time, or ongoingly, as specified by the field *continous*.

Ongoing actions will be interrupted when new ones are accepted.  

Additionally, the node publishes a Serial command to the microcontroller which drives the 
tether-guiding carriage, matching the reel's
angular velocity and direction. The serial device is expected to be named /dev/arduino_nano. 
