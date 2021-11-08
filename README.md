# Search and ~~Rescue~~ Destroy

CS81: Robotics Design and Programming, Fall 2021 at Dartmouth College

## Problem:

Map an area and efficiently patrol it, while identifying and pursuing moving objects.

## Modules:

### sentry_mode
The robot stays stationary and collects historical camera data to feed into a background 
image to perform background subtraction. If it detects anomalous objects of a certain size or
larger, it will trigger the PID chaser module.

### pid_chaser
The robot attempts to keep track of the object using a camera, while using a PID controller
to route toward the center of mass of the target.