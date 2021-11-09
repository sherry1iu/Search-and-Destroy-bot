# Search and ~~Rescue~~ Destroy

CS81: Robotics Design and Programming, Fall 2021 at Dartmouth College

## Problem:

Map an area and efficiently patrol it, while identifying and pursuing moving objects.

## Modules:
Each module may be separately triggered by running the file with the same name as the outer
module.

### patroller

### restorer

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