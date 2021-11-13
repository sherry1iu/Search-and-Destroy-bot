# Search and ~~Rescue~~ Destroy

CS81: Robotics Design and Programming, Fall 2021 at Dartmouth College

## Problem:

Map an area and efficiently patrol it, while identifying and pursuing moving objects.

## Modules:
Each module may be separately triggered by running the file with the same name as the outer
module.

### patroller

### Camera

### Chaser

### restorer

## Testing:
If you have a working setup with Gazebo, you can do the following steps to view image data
and similar:

```
cd ~/catkin_ws/src
git clone https://github.com/husarion/rosbot_description
cd ~/ros_workspace
rosdep install --from-paths src --ignore-src -r -y
catkin_make
roslaunch rosbot_description rosbot_rviz_gmapping.launch

# that last command will show some red text. That's OK -- it loads the right world
```

From here, you have a functional rosbot in a world.
If you do
```
rqt
Plugins->Visualization->Image View
then select /camera/rgb/image_raw from the dropdown
```
you will be able to see image data.

Now run
```
./src/patroller/patroller.py
./src/restorer/restorer.py
./src/utilities/tf_map_publisher.py
```

## Modifying the World:
Once you've inserted objects into the world you launched with
`roslaunch rosbot_description rosbot_rviz_gmapping.launch`, you can hit 
File->Save As. You may find that the Gazebo window becomes greyed out and unresponsive.
This is because the file saving window actually shows up *behind* the gazebo window.

You can access the file saving window by minimizing and then maximizing the gazebo window.
However, in order to reload the file saving UI, you will need to repeat the 
minimization+maximization step.

I recommend you type out `turtlebot_playground.world` and hit enter. This will save the file
into your home directory. You should then copy it over to 
`catkin_ws/src/rosbot_description/src/rosbot_gazebo/world`. It will override an existing world.
When you re-launch the simulation, it will retain your changes.