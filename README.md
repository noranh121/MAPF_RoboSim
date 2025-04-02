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

To install pygame, type the following command in the terminal

```
pip install pygame
```

### `OrderedSet`

To install ordered set, type the following command in the terminal

```
pip install orderedset
```

### `Heapdict`

To install heapdict, type the following command in the terminal

```
pip install heapdict
```

### `Argparse`

To install argparse, type the following command in the terminal

```
pip install argparse
```

### `Turtlebot3` 
````
https://ros2-industrial-workshop.readthedocs.io/en/latest/_source/navigation/ROS2-Turtlebot.html
````


## run project (for now)
* cloning the project
```
git clone https://github.com/noranh121/MAPF_RoboSim.git
cd MAPF_RoboSim/ros2_ws
 ```
* run the server
```
python3 app/app.py
```
* upload benchmark (example: MAPF_RoboSim/ros2_ws/src/Implementation-of-A-star-algorithm-for-path-planning-of-Turtlebot-in-an-obstacle-environment/a_star_tb3/benchmarks/benchmark.txt)
* upload start_end points (example: MAPF_RoboSim/ros2_ws/src/Implementation-of-A-star-algorithm-for-path-planning-of-Turtlebot-in-an-obstacle-environment/a_star_tb3/benchmarks/test.txt)
* to upload stats click the button "upload stats" and you can find the stats in this path MAPF_RoboSim/ros2_ws/uploads/exported_data.txt (it will change so the user can choose the location to download stats)

NOTE: The start_end format will change, but this is the current format that the system accepts for now.
