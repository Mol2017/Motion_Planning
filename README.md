# Motion Planning For UAV
The project is a C++ implementation of old-school motion planning pipeline (path finding + optimization). 
- [x] [RRT* algorithm](trajectory_generator/src/rtt_searcher.cpp) path finding based on OMPL library
- [x] [RDP algorithm](trajectory_generator/src/rtt_searcher.cpp) minimize waypoints based on divide & conquer
- [x] [Minimum-snap algorithm](trajectory_generator/src/trajectory_generator_waypoint.cpp) trajectory generation based on QP problem
- [x] [Trajectory replanning](trajectory_generator/src/trajectory_generator_node.cpp) trajectory replanning


## Compile & Run
```bash

sudo apt-get install cmake libopenblas-dev liblapack-dev libarpack-dev libarpack2-dev libsuperlu-dev

# Compile and run basic program
cd ./catkin_ws
catkin build
roslaunch trajectory_generator demo.launch
```

## Result Graphs
![RRT*_Result](graphs/result.png)
