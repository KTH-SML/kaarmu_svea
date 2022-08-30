# ITRL+SML Vehicle Platform ROS messages
A collection of custom vehicle-related ROS messages used in our different
platforms. Currently, this repos consists of core message types and does
not contain project-specific messages (this could change if it needs to).
The hope is for this repo to be used as a submodule so ROS messaging can be
better standardized for data commonly used in many different projects.

## SVEA msgs
Currently available messages:
1. `lli_ctrl.msg` - for communicating control inputs w/ the low level interface
2. `lli_encoder.msg` - for communicating the wheel encoder values w/ the low level interface
3. `VehicleState.msg` - 2D bicycle state suitable control of vehicles

## RCVE msgs

## Logitech Wheel msgs
Currently available messages:
1. `LogitechWheelCmd.msg` - generalized message from logitech wheel inteded for SVEA, RCVE, and CARLA

## CARLA Simulator msgs
