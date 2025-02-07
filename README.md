## Campus Planner
Implementation of the Simple Planning Algorithm to find the optimum path plan for the university campus locomotion for the robot by using the A* Search Algorithm. 

## Requirement
ROS Noetic

Ubuntu 20.04

Dependencies `catkin`, `rviz`

ROS installation Tutorial with Virtual Box: [Tutorial by Moster](https://www.youtube.com/watch?v=ZEfh7NxLMxA&t=2s).


## Implementation
We need to begin with the project folder, then create a src folder within and we can use the command `catkin_make` in order to create the workspace.
Then we can create a node handle for the planner. by using the below below 
```
catkin_create_pkg campus_planner roscpp nav_msgs geometry_msgs
```
the navigate to `campus_planner`/ `src` and create a cpp file 
```
touch campus_planner_node.cpp
```
or create the .cpp file manually. then we need to edit the `CMakeLists.txt` file by uncommenting the executable and target links.

The subscribed topics are `initial_pose`, `goal`, `map`, `visualization_marker`

The A* Search Algorithm is implemented by the following [reference](https://github.com/zang09/AStar-ROS)

1. Initialization:
Start the A* algorithm with AStarSearch.
Use a priority_queue (ordered by `fCost` = `gCost` + `hCost`) to manage nodes.
Maintain parentX and parentY arrays to track parent nodes for path reconstruction.
Use `closedList` to mark visited nodes.
Create a start node with `gCost` = 0 and hCost (Manhattan distance) and add it to the queue.

2. Algorithm Execution:
Loop until openList is empty or the goal is reached.
Extract the node with the lowest `fCost` as the current node.
If it matches the goal, reconstruct the path using parentX and parentY.
Otherwise, mark it as visited and evaluate neighbouring nodes.

3. Neighbor Evaluation:
Check if the neighbour is within map boundaries and not an obstacle (isValid).
If valid and unvisited, calculate gCost and hCost, then add it to openList, storing its parent.

4. Path Reconstruction:
Once the goal is reached, backtrack using parentX and parentY and publish the path for visualization in RViz.
If no path is found, return failure.
Obstacle-Aware Cost Adjustment:

Use computeObstacleDistances to assign higher costs to cells near obstacles.
Modify movement cost by adding an inverse obstacle distance weighted by a multiplier to prioritize safer paths.

## Experiment
Initialize the source and also make sure to re-run `catkin_make` after re-editing the code
```
source devel/setup.bash
```
To run the code I have created .launch file we can run the code by using the following command. which contain the necessary commands
```
roslaunch campus_planner campus_planner.launch
```
--------------OR------------------------
Manually run
```
roscore
```
to run the node file
```
rosrun campus_planner campus_planner_node
```
to run `map_server`
```
rosrun map_server map_server map.yaml
```
And finally for the visualization
```
rviz
```
then manually add the subscribed topics to see the result. 
``



