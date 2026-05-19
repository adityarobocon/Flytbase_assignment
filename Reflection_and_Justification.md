
### Discuss your design decisions and architectural choices. 

1.Code kept relatively simple due to time constraints and college workload 
2.drone path is divided into multiple waypoints with path between waypoints being linear for easier computation and low complexity 
3.rather than continously checking if 2 drone are within crash radius ,path of drones is modeled using relative kinematics so as to definitely find if there will be  aconflict or not 
4.make_segments()	Converts waypoints into motion segments
get_pos()	Computes drone position at time t
check_overlap()	Detects spatial-temporal conflicts
run_checks()	Orchestrates simulation and visualization

### Explain how spatial and temporal checks were implemented.
Spatial Checks

The spatial conflict logic is based on the Euclidean distance between drones in 3D space.

The distance formula used is: d = sqrt{(x_2 - x_1)^2 + (y_2-y_1)^2}
function used - def dist(w1, w2):

Temporal Checks

The system performs temporal validation before checking spatial overlap.

Inside check_overlap():

start = max(seg1["start_time"], seg2["start_time"])
end = min(seg1["end_time"], seg2["end_time"])

### Describe how you leveraged AI tools to complete this assignment.
AI tools were used mainly as a coding assistant during the development of this assignment. The overall logic, structure, and approach for drone movement, collision detection, and visualization were first decided manually. After deciding the logic, AI was used to help write and refine functions such as waypoint segmentation, position calculation, overlap checking, and animation handling.

### Describe any AI integration, if applicable.
not applicable

###  Provide a testing strategy and discuss edge cases.

1.First test each function one by one if its give expected outputs using print statements (compare with what was calculated manually to the output printed on the console)
2.then try simulating primary drone and after that start adding secondary drones one by one 
3.Now add such paths so as they would collide , and see to it if the simulation shows correct collison time and coordinates (compare with manually calculated ones)

edge cases:

1.Zero-Length Segments

Handled like this:
if d == 0:
    continue

2.Parallel Motion

Handled like this:
if A == 0:

3.Exact Boundary Collision

Handled like this:
if c_start <= c_end:

4.Empty Waypoint Lists

Handled like this:
if not p_segs:


### Explain what would be required to scale the system to handle real-world data from tens of thousands of drones.

now its comparing pairwise so time complexity will increase asn O(n^2)
1.Only run the function start checks if 2 drones are within a boundry threshold(this data will be polled )
2.Use of Distributed Computing 
3.Gpu acceleration for parallel computing as it is mostly maths calculations can be offloaded to gpus for faster computing 
4.Simultaneous running systems which check for coliisons for redundancy 
