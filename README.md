# MAPF_RoboSim

## installations necessary 

1 first install wsl 

settings -> windows features -> make sure these are checked
v  virtual machine platform
				v hypervisor platform
				v windows subsystem for linux
microsoft store: ubuntu 22.04 wsl

2 install ros 2

install
````
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html 
````
workspace
````
https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html
````

this might help:
````
https://medium.com/@nilutpolkashyap/setting-up-turtlebot3-simulation-in-ros-2-humble-hawksbill-70a6fcdaf5de 
````

3 install gazebo

https://stackoverflow.com/questions/67302265/gazebo-11-does-not-run 

4 install these

### `Pygame`

<br />To install pygame, type the following command in the terminal

```
pip install pygame
```

### `OrderedSet`

<br />To install ordered set, type the following command in the terminal

```
pip install orderedset
```

### `Heapdict`

<br />To install heapdict, type the following command in the terminal

```
pip install heapdict
```

### `Argparse`

<br />To install argparse, type the following command in the terminal

```
pip install argparse
```

### `Turtlebot3` 
````
https://ros2-industrial-workshop.readthedocs.io/en/latest/_source/navigation/ROS2-Turtlebot.html
````


## cloning project (for now)
 * git clone 
 * cd ros2_ws
 * colcon build
 * source /opt/ros/humble/setup.bash
 * source ~/MAPF_RoboSim/ros2_ws/install/setup.bash
 * ros2 launch a_star_tb3 empty_world.launch.py goal_x:=5 goal_y:=0 start_x:=0 start_y:=0.25 RPM1:=40 RPM2:=20 clearance:=100 
