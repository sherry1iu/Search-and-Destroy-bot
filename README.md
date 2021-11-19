# Search and ~~Rescue~~ Destroy

CS81: Robotics Design and Programming, Fall 2021 at Dartmouth College

## Problem:

Map an area and efficiently patrol it, while identifying and pursuing moving objects.

## Modules:
Each module may be separately triggered by running the file with the same name as the outer
module.

### Graph Server (voronoi)

Generates a topological map of the environment from an occupancy grid. See the [module-specific README](./src/voronoi/README.md) for details.

### patroller

### Camera

### Chaser

### restorer

## Environment Setup (Gazebo):
If you have a working setup with Gazebo, you can do the following steps to view image data
and similar:

Pip Setup:
```
pip install scipy
pip install scikit-image
```

Catkin Setup (Assumes working catkin_ws):
```
cd ~/catkin_ws/src
git clone https://github.com/husarion/rosbot_description
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source ~/catkin_ws/devel/setup.sh
roslaunch rosbot_description rosbot_rviz_gmapping.launch

# that last command will show some red text. That's OK -- it loads the right world
```

From here, you have a functional rosbot in a world.
If you do
```
sudo apt-get install ros-melodic-rqt ros-melodic-rqt-common-plugins
source /opt/ros/melodic/setup.bash

rqt
Plugins->Visualization->Image View
then select /camera/rgb/image_raw from the dropdown
```
you will be able to see image data.

Now run (in separate terminals: you will need 7 for just the nodes. All commands should be run from the root-level directory of this repository)
Publish the map
```
./src/utilities/read_and_publish_map.py
```
Set up the graph server
```
./src/voronoi/graph_server.py
```
Run the LIDAR module
```
./src/detector/lidar_detect.py
```
Run the camera module
```
cd ./src/detector
./camera_detect.py
```
Run the chaser module
```
cd ./src/chaser
./chaser.py
```

Run the patroller module
```
cd src/patroller
./patroller.py
```
Run the restorer module
```
cd src/restorer
./restorer.py
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

## Building the Map
```
cd ~/catkin_ws/src
git clone https://github.com/marinaKollmitz/gazebo_ros_2Dmap_plugin
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make
```

Add this within the <world></world> tags of the world file.
It will cause the robot to create a map from scan data and publish on the /map topic.
``` 
<plugin name='gazebo_occupancy_map' filename='libgazebo_2Dmap_plugin.so'>
    <map_resolution>0.1</map_resolution> <!-- in meters, optional, default $
    <map_height>0.3</map_height>         <!-- in meters, optional, default $
    <map_size_x>10</map_size_x>          <!-- in meters, optional, default $
    <map_size_y>10</map_size_y>          <!-- in meters, optional, default $
    <init_robot_x>0</init_robot_x>          <!-- x coordinate in meters, op$
    <init_robot_y>0</init_robot_y>          <!-- y coordinate in meters, op$
</plugin>
```

## Comparing with Random Walk and A*
Our method traverses the graph in 5min, 10sec

You can run a random walk with
```
./src/test_drivers/random_motions.py 

```
The random walk took 15 mins, 30 seconds, and failed to explore the center of the map and the room at the bottom.