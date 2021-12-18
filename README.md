# Motion Planning For UAV
The project is a C++ implementation of old-school motion planning pipeline (path finding + optimization). 
- [x] [RRT* algorithm](trajectory_generator/src/rtt_searcher.cpp) path finding based on OMPL library
- [x] [RDP algorithm](trajectory_generator/src/rtt_searcher.cpp) minimize waypoints based on divide & conquer
- [x] [Minimum-snap algorithm](trajectory_generator/src/trajectory_generator_waypoint.cpp) trajectory generation based on QP problem

## Compile & Run
```bash
# Compile and run basic program
cd ./catkin_ws
catkin build
cd ./catkin_ws/src/grid_path_searcher/launch
roslaunch demo.launch
```

## Result Graphs
![RRT*_Result](graphs/result.png)
