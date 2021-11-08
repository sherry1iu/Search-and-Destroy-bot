# Search and ~~Rescue~~ Destroy

CS81: Robotics Design and Programming, Fall 2021 at Dartmouth College

## Problem:

Map an area and efficiently patrol it, while identifying and pursuing moving objects.

## Modules:
Each module may be separately triggered by running the file with the same name as the outer
module.

### sentry_mode
The robot stays stationary and collects historical camera data to feed into a background 
image to perform background subtraction. If it detects anomalous objects of a certain size or
larger, it will trigger the PID chaser module.

### pid_chaser
The robot attempts to keep track of the object using a camera, while using a PID controller
to route toward the center of mass of the target.

### Testing:
If you have a working setup with Gazebo, you can do the following steps to view image data
and similar:

```
cd ~/catkin_ws/src
git clone https://github.com/husarion/rosbot_description
cd ~/ros_workspace
rosdep install --from-paths src --ignore-src -r -y
catkin_make
roslaunch rosbot_description rosbot_rviz_gmapping.launch

# that last command will fail. That's OK -- it loads the right world
```

From here, you have a functional rosbot in a world.
If you do
```
rqt
Plugins->Visualization->Image View
then select /camera/rgb/image_raw from the dropdown
```
you will be able to see image data.