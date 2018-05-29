Lab 6
=====

## Overview and Motivations - [Rose Wang]

The main focus for this week’s lab was path planning and control. Path planning and control are vital elements to assist in autonomous car navigation in dynamic, unstructured, and open environments. The proposed system leverages the concept of rapidly exploring random trees (RRTs) for path planning and pure pursuit for simultaneous tracking and adaptive to sensor feedback from last lab’s particle filter localization.

The lab report discusses discuss our technical approach to path planning with an augmented RRT algorithm (RRT*) and pure pursuit. Following, it demonstrates implementation of both individually in simulation and on the RACECAR platform, then the combined implementation of real-time path planning and execution on the RACECAR platform. Finally, the report provides an analytical evaluation of the chosen path planning algorithm and pure pursuit accuracy. 


## Proposed Approach 

### Initial Setup - [Rose Wang]

Path planning and trajectory tracking depend on a predefined start and end goal, which together can define a path. Given a start and goal point in a known map environment, path planning generates an optimized path trajectory. In turn, trajectory tracking takes in a desired path, and uses a lookahead distance to determine its motion control under the Ackerman steering model. The setup for this lab additionally included a safety controller to prevent physical damage caused by unexpected obstacles. 

### Technical Approach - [Marwa Abdulhai, David Klee, Austin Floyd, Rose Wang]

<b> Path Planning </b> 
Path planning uses the location of the car and knowledge of obstacles in the map to generate an obstacle-free path that the car can follow given its differential constraints. 

To implement path planning, a path planning algorithm was selected to allow the car to autonomously travel to a specified goal point. The rapidly exploring random tree (RRT*) algorithm was chosen out of A*, DFS, BFS, Dijkstra, and PRM, the reasoning of which follows in the experimental evaluation section of the lab report.

A rapidly exploring random tree (RRT) is an algorithm designed to efficiently search nonconvex high dimensional spaces by randomly building a space filling tree. The algorithm is inherently biased to grow towards large unsearched areas and can easily handle problems with obstacles and differential constraints, which is essential in a case where the car must navigate autonomously through the stata basement while avoiding obstacles such as humans and walls. RRT does not, however, guarantee an optimal solution. For this reason, the necessary modifications were made to implement RRT*, a similar algorithm that does converge towards an optimal solution. 

RRT runs the following: Given a map, a start node, and a goal point, the algorithm samples a random point on the map coordinate frame $$ x_{\text{random}} $$ which will be referred to as test point or random point interchangeably.  It finds the nearest node $$ x_{\text{near}}$$ on the tree to the new random point by computing the distance to each tree node and selecting the node that has the smallest euclidean distance away from the random point. If the generated path between those two points is free of any obstacles (in other words, if the path between them does not pass through a non-permissible region), then a new node denoted $$ x_{\text{new}} $$ that is a fixed step size from the random point is added to the tree using a steer function that takes into account the angle between the two points such that the increment in the x and y direction are as follows:

$$ x_{new_{x}} = x_{near} + STEP_SIZE* cos(\theta)$$

$$x_{new_y} = x_{near} + STEP_SIZE* sin(\theta) $$


<p align="center">
  <img src="assets/images/lab6/path_planning/rrt.png" img width="400" height="300">
</p>

<p align="center">
  <b> Figure 1: Illustrating the RRT Algorithm </b>
</p>

The metric to test whether a point lies is a non-permissible region is as follows: a line is drawn through the random node and potential near node (with a respective slope and y intercept) and a sample of 20 points along the line are tested one by one to see if any lie in a non-permissible region (having pixel value not equal  to zero in array of map). If the path does pass through a non-permissible region, the algorithm generates another random point and runs through the aforementioned process once again till very close to the goal point (taken to be 0.5 m in this scenario), in which case the nearest node is selected to be the goal node. 

<p align="center">
  <img src="assets/images/lab6/rrt_branch_regular.PNG" img width="460" height="300">
</p>

<p align="center">
  <b> Figure 2,3: RRT Branched Nodes in Red, RRT Final Path Displayed in White </b>
</p>

However, this algorithm is naive in the way it selects a vertex at random to add to the ever- growing tree causing the path to look crooked or jagged. To overcome this problem, the  successor to RRT, RRT*, is implemented which adds additional selective metrics to determine the nearest point to the new node and improves path quality by introducing tree wiring.  Firstly, RRT* inspects each one of the nodes laid within the neighborhood of the newly added random node defined by the formula 

<p align="center">
  <img src="assets/images/lab6/rrt_equation.png" img width="460" height="300">
</p>

Where d is the dimension of the search space and $$\gamma$$ is the planning constant based on the environment. It selects the nearest node to the $$x_new$$ node to replace it as the new node. Secondly, rewiring then rebuilds the tree by checking if the cost to each node within this radius of area k is less through $$x_{new}$$ as compared to their older costs and changes each node’s parent to be $$x_{new}$$ if this is true. As the number of iterations increase, RRT* improves its path cost gradually due to its asymptotic quality unlike RRT which does not improve it jagged path. 

The trees produced by RRT and RRT*, with the former shown in Figure 3, often produce branches than touch, or come very close to touching, the permissible region boundaries. Figure 4 shows a resulting path planned by RRT, which includes many points nearly touching the permissible region boundary. In practice, this means that the car’s path will cut corners closely, come within centimeters of walls and other obstacles, and likely crash. There were a number of ways to approach this obstacle problem – one includes “thickening” obstacles on the map through a method called morphological dilation. An example of an element being dilated with 3x3 padding (a dilation parameter of 9) is given by the matrices below.

<p align="center">
  <img src="assets/images/lab6/matrix.png" img width="460" height="300">
</p>

<p align="center">
  <b> Figure 4: Morphological Dilation. Given a matrix (left), dilation can be performed by changing the neighboring elements of non-zero entries. The resulting matrix (right) is created given some dilation parameter (in this case 9) </b>
</p>

By applying morphological dilation to an array representing the Stata basement map, all regions representing walls and other obstacles can be enlarged. Then, permissible regions can be computed from this new dilated map. The path planning algorithm can use these newly defined permissible regions as it samples points. This ensures that the car’s planned path never gets too close to walls.

<p align="center">
  <img src="assets/images/lab6/dilated_map.png" img width="460" height="300">
</p>

<p align="center">
  <b> Figure 5: Morphological Dilation. Given a matrix (left), dilation can be performed by changing the neighboring elements of non-zero entries. The resulting matrix (right) is created given some dilation parameter (in this case 9) </b>
</p>

<b> Pure Pursuit Control </b>

In this lab, trajectory tracking is implemented using pure pursuit control.  Given the car’s location and the waypoints in a trajectory, a pure pursuit controller returns a steering angle command to move the car along the trajectory.  First, a circle with radius called lookahead distance is constructed around the car’s location.  Then, any intersections between the circle and the trajectory are found (refer to Figure X).  The intersection that is farther along the trajectory is chosen as the point to pursue.  Using the ackermann drive kinematic equations, it is possible to solve for the steering angle to intersect the point to pursue in closed form.  The steering angle must be bounded to reflect constraints imposed by the physical system (RACECAR platform has max steering angle ~0.30 radians).

<p align="center">
  <img src="assets/images/lab6/pursuit_lookahead.png" img width="460" height="300">
</p>

<p align="center">
  <b> Figure 6: Finding Point to Pursue.  Here the lookahead circle intersects the trajectory in two places.  The star, indicating the point to pursue, is the intersection that lies farther along the length of the trajectory.
 </b>
</p>

There is only one parameter within the pure pursuit controller that must be tuned: the lookahead distance.  A small lookahead distance means the car quickly approaches the local trajectory, but can cause the car to oscillate around the trajectory.  A large lookahead distance will smooth over the trajectory but takes a long time to reach the trajectory.  This behavior is shown in Figure X.  A discussion of tuning lookahead distance will be discussed in the Experimental Evaluation section.

<p align="center">
  <img src="assets/images/lab6/three_traj.png" img width="460" height="300">
</p>

<p align="center">
  <b> Figure 7: Path Following with Different Lookahead Distances.  The lookahead distances are 0.5, 1.5 and 4.0 m for a,b,c, respectively.  For lookahead distance of 0.5 m, the car closely follows the path (shown in red) but fails to anticipate a sharp turn.  For a lookahead distance of 4.0 m, the car ignores a lot of details of the path.  
 </b>
</p>

### ROS Implementation - [Marwa Abdulhai, Rose Wang]

The basic architecture of the system is as follows: 
<b> BuildTrajectory class </b> listens to points published on Rviz to build a trajectory. Specifically, the 2D Nav Goal tool on Rviz is used to set the start and end node locations on the map coordinate frame. These two Marker objects are published by the class to the topics /start_point and /end_pose respectively. They are also added to a LineTrajectory object and saved to the file system.

<b> The LoadTrajectory class </b> loads the saved trajectory in the file system and publishes it to the ROS topic /trajectory/current as a LineTrajectory object. 

<b> The PathPlanner() class </b> is called and subscribes to the topic /trajectory/current which contains the start and end points. PathPlanner() passes the parameters, a newly dilated map, and an array containing the permissible regions on the map to the function rrt_graph_determine_path(), an instance method of the class RRT. 

rrt_graph_determine_path() returns the new LineTrajectory object containing points along the path from the start node to the end node and publishes it for display on rviz to the topic /CURRENT_PATH/path. It displays the intermediate nodes of the branched RRT* path to the topic /rrt_visual for visualizing the path generation. 

An additional data structure, <b> Tree </b>, was constructed to keep track of the points that have been searched thus far, the parent of each point, and the cost of each point calculated as a function of the distance it is away from the start node. This allows for RRT* to update the cost of each node efficiently at every iteration of adding a new point to the trajectory. 

## Experimental Evaluation 

The following section discusses the methods used in testing the path planning algorithm of choice and pure pursuit for trajectory tracking. Specifically, each part elaborates on the metrics used for testing procedure and the methods for optimization given the metrics. The testing procedure additionally elaborates on the choice of path planning algorithm. Lastly, the section provides an analytical review for the results generated from the discussed testing and optimization procedures. 

### Testing Procedure - [Sean Patrick Kelly, Rose Wang, Marwa Abdulhai]

<b> Path Planning </b>
For the purposes of efficient testing, RRT* was implemented in small increments to ensure the functionality of each underlying function. RViz was especially helpful in visualizing the graph structure, and providing indications of potential areas for optimization. The graphics included below were generated from RViz. 

1. The success of the path planning algorithm is measured with the following metrics: 
2. The time it takes for the algorithm to generate a single path from start to finish.
3. The number of intermediate nodes or branches generated by the algorithm.
4. The distances between generated branches and given map obstacles.

The testing procedure explored rate of map dilation, and measured analytically how map dilation allowed for improved path generation. Improvements and selective tuning to the path trajectory were primarily evaluated on how the algorithm deals with corners and the distances between the path and its closest obstacles. The below figure demonstrates visually the applications of map dilation to the provided map of the Stata basement. 

<p align="center">
  <img src="assets/images/lab6/map_dilation_stages.png" img width="400" height="575">
</p>

<p align="center">
  <b> Figures 8, 9, 10 Dilated Maps. The outputted map of the Stata basement following dilation of amounts 1 (top), 16 (middle), and 25 (bottom). Note that a dilation parameter of 1 corresponds to no change in the original map.
 </b>
</p>

The parameters selected to be most helpful in tuning included the minimum distance away the algorithm should be prior to determining a complete path between start and goal points, the maximum number of point queries to be made in a permissible region, the step size distance between a newly added point and the nearest neighbor point in the trajectory, and the maximum threshold of iterations for random sampling, as opposed to sampling the goal point. 

The range of changes for: 
1. Minimum distance: This represents the minimum distance the new point added to the 
trajectory needs to be away from the goal node in order for the path to be complete. We 
set this to be 0.1 meters for accuracy. 

2. Maximum number of point queries: Based on experimentation, we found that there 
were tradeoffs to setting this value to an extremely high number (in the order of billions) 
to really small (in the order of thousands). These random points are all generated all at 
once (before the path planning algorithm begins), so there may be issues if the value is 
too small (in which case the algorithm fails to find a path as it as no more points to 
sample) or the value is too big (in which case there is a run-time cost incurred in the 
beggining of the algorithm). We thus found 100,000 to fit our needs best. 

3. Step size distance: This was set to be 1 meter to avoid overshooting the path at 
corners and thus causing a collision.  

4. Maximum threshold of iterations (end bias): For every 100 points, 1 will be the end    point. We chose this number as when the tree nears the goal state, there is an increased likelihood of it extending to the goal state. Without goal biasing, the tree may come close to the goal but fail to achieve it. 

<b> Path Planning: Choice of Algorithm </b>
The planning algorithms initially under consideration were Dijkstra, A*, RRT(*) and PRM. Dijkstra, A* and PRM (using a Dijkstra planner) are grid-based search algorithms, and RRT(*) is a sampling-based search algorithm. 

By its algorithmic construction, RRT provided an advantage over all other algorithms because it is not grid-based and provides algorithmic flexibility when working under real-world conditions. Grid based algorithms reasonably handles lower dimensional problems because it depends on a more discretized state space. The discretized space is achieved by overlaying a grid on top of a configuration space. However, with high dimensional systems under complex constraints or mimicking real environmental factors, exact motion planning is computationally intractable with grid search algorithms. Therefore, the choice of algorithm naturally tended towards sampling-based search algorithms, such as RRT. RRT is a method of easily handling problems with obstacles and differential constraints including nonholonomic and kinodynamic, which were applicable to the motion of RACECAR platform.

<b> Pure Pursuit </b>

The pure pursuit trajectory tracking algorithm was tested over the course of its development by using RViz to simulate the car and visualize the trajectory, the closest point to the path, and the lookahead intersection point (Fig. X).  This allowed for rapid debugging and qualitative performance evaluation of the algorithm, since qualitative success was required before quantitative evaluation could be conducted.  The former was defined by three key requirements:
The published nearest point to the trajectory was accurate and traversed the trajectory without discontinuity.
The lookahead pursuit point remained in front of the closest point and traversed the trajectory without discontinuity.
The steering controller reacted correctly to turns in the trajectory.

<p align="center">
  <img src="assets/images/lab6/twopointscreenshot.png" img width="400" height="300">
</p>

<p align="center">
  <b> Fig. 11: Visualization of Pure Pursuit Logic. The trajectory path (red line), the look-ahead intersect point (blue dot), and closest point on the trajectory to the car (yellow dot).  In order for qualitative success, the dots must have traversed the trajectory without discontinuity, with the blue “pursuit” point remaining in front of the yellow “location” point.
 </b>
</p>

The completed pure pursuit algorithm was evaluated using absolute position error over time, calculated by taking the absolute closest distance to the trajectory at every time step.  This performance metric was run on three different trajectories at a lookahead distance of 1.5 meters (Fig. X): a straight path, a jagged sinusoidal path, and a smooth path around corners.  The straight path trajectory provided a baseline for data analysis and would highlight the existence of integral error.  The jagged path demonstrated the degree to which corners are cut by the control algorithm and the reaction to trajectory changes tighter than the maximum steering angle.  The fine corner turning path demonstrated the algorithm’s ability to follow trajectories comprised of very small line segments.

<p align="center">
  <img src="assets/images/lab6/lab6threetraj.png" img width="460" height="300">
</p>

<p align="center">
  <b> Fig. 12: Pure Pursuit Evaluation Trajectories. (A) The straight line trajectory, (B) the jagged sinusoidal trajectory, and (C) the smooth corner trajectory.  Note that each trajectory starts at the bottom near the grid and finishes at the top in (A) and (B) and at the right most point in (C).
 </b>
</p>


In addition to testing the algorithm on three trajectories, tuning the lookahead distance parameter was done by evaluating the varied performance of the pure pursuit algorithm following a trajectory generated by our path planning algorithm.  Since the controller should be optimized for the shape of path expected from our RRT* implementation, we did not use the previous custom trajectories.  By iterating through a range from 0.5 meters to 4.0 meters, an optimal lookahead distance was determined based on total position error accumulation and whether or not the lookahead distance caused the car to lose the trajectory.

This performance analysis was repeated using the real RACECAR platform in order to demonstrate the effect added positional uncertainty had on the performance of the algorithm.  

### Results - [Sean Kelley, Lucas Novak, Rose Wang]

<b> Path Planning </b>

Running RRT in a Straight Line
The first way we decided to evaluate our algorithms was to make them find a path of two points straight down a hallway (seen below in figure X). We wanted the ending point to be within .1m of the goal point and the step size to be 1 meter. We ran 10 tests of RRT, RRT* and RRT* with dilation of the map. A big thing we quickly notice was that once we decreased the minimum distance to be .1 meters (which was chosen so the car would end up within a car length of its intended spot) the steer function could prevent RRT from quickly converging to the goal. The steer function behaved the same in every algorithm, moving the tree one step size from a near point to a sampled point. This meant that if the test point was ever close to the goal but not one step size away from the goal it would overshoot the goal but getting slightly close. While RRT* would avoid these marginally closer points because they began to cost much more, RRT did not have anything in place to prevent this. We never improved the functionality of this as we quickly moved on to RRT* instead of RRT as it provided a path much closer to the optimal path. If we wanted to fix it we could either modify steer or start another tree coming from the goal to the start and have them meet up, eliminating the need for convergence. 

Note 1 pixels of dilation is about .09 meters.

<p align="center">
  <img src="assets/images/lab6/path_planning/straight_path.png" img width="460" height="300">
</p>

<p align="center">
  <b> Fig. 13: Path of 74 meters used for straight line test
 </b>
</p>

Nodes Generated
The first metric we used to evaluate our algorithms was the number of nodes that were sampled and then added to the tree to help generate the path. As mentioned above many of the trials of RRT generated a large number of nodes as they took a very long time to converge. However, the times were this wasn't a problem RRT generated about the same amount of nodes as RRT* (with or without dilation). It also appears that using dilation reduces the amount of nodes generated on the tree, which makes sense as there are now less possible valid positions for nodes to exist.

<p align="center">
  <img src="assets/images/lab6/path_planning/1.png" img width="460" height="450">
</p>

<p align="center">
  <b> Table 1: The number of nodes generated by different types of RRT algorithms for trying to find a path within .1m of the goal.

 </b>
</p>

<p align="center">
  <img src="assets/images/lab6/path_planning/rrt_straight_all.png" img width="460" height="300">
</p> 
<p align="center">
  <img src="assets/images/lab6/path_planning/rrt_straightline_star_only.png" img width="460" height="300">
</p>

<p align="center">
  <b> Figures 14,15: Boxplots of the number of nodes generated by different types of RRT algorithms (data in table 1). 14 is off all the algorithms and 15 is only of RRT* and RRT* with dilation to improve clarity.

 </b>
</p>

Distance of Generated Trajectories
The next metric we used to evaluate them was the distance of the trajectory the different algorithms generated with the logic being the closer to the straight line distance of 74 meters the better the path the robot would generate. We saw that RRT was significantly slower than RRT*. Dilation also seemed to reduce the distance of the line generated though not by much.

<p align="center">
  <img src="assets/images/lab6/path_planning/2.png" img width="460" height="500">
</p>

<p align="center">
  <b> Table 2: The distance of the generated trajectories of the different types of RRT algorithms trying to find a path within .1m of the goal.

 </b>
</p>

<p align="center">
  <img src="assets/images/lab6/path_planning/rrt_straight_dist.png" img width="460" height="500">
</p>

<p align="center">
  <b> Figure 16: Boxplot of the distance of the generated trajectories (data in table 2).

 </b>
</p>

Time to Run Each Algorithm (does not include time to Dilate Map)

The final metric we used was the time it took for the algorithm to generate the path. Again RRT had a large time in general because of convergence. Unlike nodes though, when RRT quickly converged it converged 4 times faster than any RRT* time. This could potentially be applied by running our RRT many times as we only need one path to converge. Dilation also improved speed of RRT* by about 16%.

<p align="center">
  <img src="assets/images/lab6/path_planning/3.png" img width="460" height="500">
</p>

<p align="center">
  <b> Table 3: The amount of time it took to run each algorithm when trying to find a path within .1m of the goal.
 </b>
</p>


<p align="center">
  <img src="assets/images/lab6/path_planning/rrt_straight_time_all.png" img width="460" height="300">
  <img src="assets/images/lab6/path_planning/rrt_straight_star_dist.png" img width="460" height="300">
</p>

<p align="center">
  <b> Figures 17,18: Boxplots of the amount of time it took to generate the different algorithms. Figure 16 is all of them and figure 17 is only RRT* and RRT* with dilation to improve clarity.
 </b>
</p>

<i> RRT Going Around Corners and the Map </i>
After the straight line test we decided to only test RRT* with different dilations and to test sending the car around corners to the other side of the map (shown below in figure 17). We choose to test no dilation, 19 pixels of dilation, and 35 pixels of dilation (examples of maps generated with and without dilation can be seen below in figures 17,18). No dilation was chosen as a base line. 19 pixels was about the minimum amount of dilation to ensure the car would not hit the wall around tight corners on the map. 35 pixels was about the maximum dilation until part of the map become impassable. Like before we want to be within .1 meters of the goal and have a step size of 1 meter. We also used the same metrics from above to evaluate.


<p align="center">
  <img src="assets/images/lab6/path_planning/curve_path.png" img width="460" height="300">
</p>

<p align="center">
  <b> Figure 19: Path of car going around corner and across map. Manhattan distance is 107 meters.
 </b>
</p>

Nodes Generation
Similar to the straight line dilations decrease the number of nodes generated on the trade for the same reason. What is surprising is how much lower dilating with 19 pixels instead of 35. It could be because at some point of dilating it starts impeding paths through narrow corridors. It could also just be because of chance. 


<p align="center">
  <img src="assets/images/lab6/path_planning/4.png" img width="460" height="500">
</p>

<p align="center">
  <b> Table 4: Data of the number of nodes generated by RRT* with different levels of dilation for the curve path.
 </b>
</p>


<p align="center">
  <img src="assets/images/lab6/path_planning/rrt_curve_nodes.png" img width="460" height="300">
</p>

<p align="center">
  <b> Figure 20: Boxplot of the number of nodes generated by RRT* with different dilations for the curved line. Data can be seen in table 4.
 </b>
</p>

Distance of Generated Trajectories
The distances of the routes generated by RRT* are all about the same regardless of their dilation. Dilation does seem to lower the distance by a little bit (about 1 meter) but future testing is needed to see if this difference is meaningful (i.e. the 1 meter represents a lot less curvature, mildly less curvature or not relation).

<p align="center">
  <img src="assets/images/lab6/path_planning/5.png" img width="460" height="300">
</p>

<p align="center">
  <b> Table 5: Data of the distance in meters of the generated trajectory of RRT* with different dilations on the curved path.
 </b>
</p>

<p align="center">
  <img src="assets/images/lab6/path_planning/rrt_curve_dist.png" img width="460" height="300">
</p>

<p align="center">
  <b> Figure 21: Boxplot of the distance of the generated trajectories by RRT* with different dilations. Data seen in table 5.
 </b>
</p>

Time to Run Each Algorithm (does not include time to dilate map)

Overall time of dilation of 19 was smaller than both no dilation and dilation of 35 with 19’s time being 20% lower than either no dilation or 35 dilation. Something interesting to note is that the minimum two times and highest three times to make a path came from 35 dilation. The could be because the higher dilation makes more narrow corridors so only a small range of values can get through. If we happen to sample from those points it will go very quick because there are less valid points than the less dilated versions but can very easily not sample and take more time. It could also just be random chance so more testing is needed.

<p align="center">
  <img src="assets/images/lab6/path_planning/6.png" img width="460" height="300">
</p>

<p align="center">
  <b> Table 6: Data of the time in seconds that it took RRT* with various dilations to run on the curved path.
 </b>
</p>

<p align="center">
  <img src="assets/images/lab6/path_planning/rrt_curve_time.png" img width="460" height="300">
</p>

<p align="center">
  <b> Figure 22: Boxplot of the time it took RRT* with various dilations to run on curved path (data seen in table 6).
 </b>
</p>

<b> RRT Map Dilation </b>

As described in the Technical Approach, the planning algorithm is able to avoid getting too close to walls and obstacles by applying morphological dilation to its map. Through trial and error, a dilation parameter of 41 was chosen for a normal test case (going around the Stata basement), but a smaller parameter of 15 was chosen for a test case in a narrow environment to avoid closing off paths. Visualization of the former is show in Figures 23-24 below.

<p align="center">
  <img src="assets/images/lab6/path_planning/rrt_without_dilation.png" img width="460" height="300">
</p>

<p align="center">
  <b> Figure 23, 24: RRT* Without Dilation, RRT* With Dilation Branching Nodes Structure: Tree Grown from RRT* with Dilation stays closer to the center of the path (right) than RRT* without (left) where the nodes are hugging the sides of the wall.
 </b>
</p>

<p align="center">
  <img src="assets/images/lab6/path_planning/rrt_path_dilation_comparison.png" img width="460" height="300">
</p>

<p align="center">
  <b> Figure 25, 26: RRT* Without Dilation, RRT* With Dilation Drawing Path Across Stata Basement: 
 </b>
</p>

<b> Pure Pursuit </b>

The graphical result of position error analysis for different trajectories in simulation using a lookahead distance of 1.5 meters and speed constant of 1.5 are shown in figures X and X.

<p align="center">
  <img src="assets/images/lab6/pos_error_straight.png" img width="460" height="300">
</p>

<p align="center">
  <b> Figure 27 Pure Pursuit on Straight Trajectory.
 </b>
</p>

<p align="center">
  <img src="assets/images/lab6/pos_error_smoothturn.png" img width="460" height="300">
</p>

<p align="center">
  <b> Figure 28 Pure Pursuit on Smooth Trajectory. 
 </b>
</p>

The position error over time (Fig. 28) for the straight trajectory (trajectory A, Fig. 28) was used to verify the accuracy of our analysis methods and to identify a possible steady state control error.  As the graph suggests, the car corrected an initial position error from the trajectory within three seconds with an overshoot of less than 0.05 meters.  For the remainder of the trajectory, the offtrack error remained zero, showing the absence of steady state error.

The position error (Fig. 28) for the smooth turn trajectory (trajectory C, Fig. 28) verifies the hypothesis that more complex geometries result in more areas with position error.  In this case, large spikes in position error correspond to turns, and small sinusoidal patterns reflect and issue in the car’s control algorithm when attempting to follow trajectories composed of many small line segments shorter than the lookahead distance.  

<p align="center">
  <img src="assets/images/lab6/pos_error_zigzag_twolookahead.png" img width="460" height="300">
</p>

The position error for multiple lookahead distances (Fig. 28) for the zig zag trajectory (trajectory B, Fig. 28) confirms the existence of a relationship between look ahead distance and pursuit accuracy.  With a greater look ahead distance, the car simply drove in a nearly straight line through the sharp curves, thereby accumulating a greater position error while finishing the trajectory faster.  Note that the trajectory was run a third time using a look ahead distance of 0.5 meters, but because the car was unable to negotiate the sharpest turn while maintaining an overshoot within 0.5 meters of the path due to physical steering angle constraints, the robot was not able to complete the path.  

An optimal lookahead distance was determined to be 1.5 meters for the car given a constant speed of 1.5 m/s.  This was calculated by finding the largest lookahead distance, thereby the inherently smoothest path, with a maximum position error of 0.15 meters so as to avoid collisions when navigating trajectories around corners.

The pure pursuit controller needed to also work reliably to follow trajectories on the RACECAR platform.  Unlike in simulation, the RACECAR platform must use a particle filter to localize itself in relation to the map and the trajectory which lies in the map’s coordinate frame.  In Table 7, the accuracy of the pure pursuit algorithm is shown for runs in simulation and on the RACECAR.

<p align="center">
  <img src="assets/images/lab6/pure_pursuit_table.png" img width="300" height="300">
</p>

<p align="center">
  <b> Table 7. Pure Pursuit Position Error in Simulation vs RACECAR
 </b>
</p>

In simulation the position error is relatively low, with a mean less than about 0.1 m.  In comparison, on the RACECAR platform, the mean position error reaches up to 0.16 m on the zigzag trajectory.  All of these data are generated using the same lookahead distance of 1.5 m so the additional position error on the RACECAR is not solely a result of cutting more corners than the simulation (Fig. 29).  Instead, the increase in position error is likely due to errors within the particle filter.  The particle filter can cause the known pose of the car to drift independent of odometry data (if it receives certain sensor data).  This drifting, or other unexpected jumps in the inferred pose of the particle cloud, is not anticipated by the pure pursuit algorithm which assumes ackermann steering constraints are placed on the motion of the car. The error could potentially be further decreased by adapting the particle filter algorithm written by the course administrators.

<p align="center">
  <img src="assets/images/lab6/traj_zig_zag_2.png" img width="460" height="300">
</p>

<p align="center">
  <b> Figure 29 Trajectory Followed in Simulation vs RACECAR.  This is the zigzag trajectory (B) and a lookahead distance of 1.5 meters was used.  The particle filter used was developed by team 3 in lab 5.  The two paths followed take on the same general shape (cutting small corners, overshooting the sharp turn in the middle) but the trajectory traced by the RACECAR is much less smooth.  This is likely due to the particle filter readjusting pose discontinuously during motion.
 </b>
</p>


## Lessons Learned 

### Technical Conclusions - [Rose Wang]

During development of path planning and trajectory tracking, the team was attentive to the following pitfalls: 
1. coordinate conversions
2. jagged path generation due to high discretization
3. edge cases for pure pursuit (e.g. multiple intersection points within the lookahead radius)
4. rate of convergence for path generation
5. vectorization with numpy
6. caching and reusing large buffers
7. implement visualization early to help debugging

Implementation and testing proved to be more effective in this week’s lab given the efforts on testing the functionality of each underlying method, before continuing onto a new task. 


### CI Conclusions 

[Lucas]: For this lab we divided up work to an extreme we had not tried before - two for pure pursuit and four for path planning. And in path planning we also divided up even more, some people to visualization, some people to path planning algorithm, some on optimizing it, one person on dilation. This was also compounded with each of us having very different schedules. I personally couldn’t do much work until the end of the week so it was weird for me both trusting the team, not that I didn’t trust them to do the work, I was just a little worried I would be perceived as not doing much work or not being able to really be effective after coming back because our workflow hadn’t been the best. In the end I had nothing to worry about. With good communication and planning we were able to really be effective on finishing everything when we needed to. I also think it is nice for everyone to know that they can trust the others to get the things done and then help bringing it together.

[David] This lab felt a lot more rushed after coming of the three week long lab 5.  For that reason, there was not as much room for hiccups or delays.  We started off the week strong.  After talking amongst our team and then consulting our TA for advice we decide to divide the team into two groups of different sizes.  Each group had a task and would delegate tasks within the group.  We agreed as a team to put more emphasis on generating data and graphs; to always be thinking about what the solution would look like and how to characterize that.  To a limited extent we achieved that goal, although not far enough in advance to be able to make changes or optimize based on the results of the analysis.  There needs to be more communication between the groups.  We wanted both tasks to be done by Saturday so we could integrate on Sunday but that did not happen until Tuesday which took away our time for the CI aspects of the class (making report and briefing).

[Austin] We placed a heavier significance on our workflow for this lab. We divided into two main groups for each of the lab’s main components, and doing so allowed us to get trajectory tracking and path planning implemented. I think that better communication within each sub-group would help to avoid some issues that came up this week, such as a point where part of the sub-group spent time discussing something that others had already discussed and decided on.

[Sean]: Our technical approach to this lab was more effective than previous labs.  We were able to divide into two teams to address the two major problems of the lab and we worked efficiently within these teams.  However, tasks could have been organized much more effectively within the sub-teams.  Poor planning and communication on a sub-team level resulted in significantly delaying subsystem integration.  In the future we need to stress the importance of finishing subtasks well before the due date so that sufficient time can be alloted for optimizing important parameters that affect the system as a whole and evaluating the performance of the completed system.

[Marwa]: For this lab our team was able to effectively divide into two groups to implement the path planning algorithm & pure pursuit strategy respectively, debugging our individual parts before integration. In the future, we could benefit from making our code better equipped for integration with other parts. In addition, our team should figure out a better workflow for transferring things onto our website.

[Rose]: Our team did a great job at dividing and executing their parts of the lab. Because we emphasized more on debugging the individual parts of the code and ensuring their functionality with visuals in RViz, we were able to easily identify where issues were derived from. We’ve definitely improved our workflow for the execution of this lab. I hope in the future, we can further work on our workflow for integrating all the materials into our website. 

## Future Work - [Marwa Abdulhai]

For this lab the RRT* algorithm was implemented using a one directional growing from the start point. The team hopes to integrate a bi-directional framework such that there are two RRT’s growing - one from the start point and the other from the endpoint. This would be immensely helpful in trying to navigate through narrow passageways where it would take the algorithm a great deal of time to find a path with dilation. 

