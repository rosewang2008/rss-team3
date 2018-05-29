Final Challenge: The Labyrinth
=====

## Overview and Motivations

This report presents an overview of our solution to The Labyrinth Final Challenge.  The objective of The Labyrinth was to navigate a specified distance away from the starting position in a maze of randomly placed obstacles, with no prior knowledge of the environment, and without collisions. This challenge was selected due to the necessity of implementing and integrating the core components needed in a robust robotic system: localization, mapping, and planning. 

The first section of the report elaborates on the setup of the challenge and our the technical approach. The report discusses our implementation of the approach, which is then evaluated by selected quantitative and qualitative metrics in the following section. Finally, the report concludes with the lessons the team learned through the final challenge.


## Proposed Approach 

The approach to this challenge consisted of dividing the tasks into four major components:
1. Generate, and continuously update, a map of the car’s environment and absolute position, using simultaneous localization and mapping (SLAM)
2. Develop a path planning algorithm to find feasible trajectories through narrow corridors to possible exit points
3. Determine areas of interest in the incomplete SLAM map in order to identify waypoints to explore to find an exit
4. Design a trajectory tracking controller that allows the car to closely follow, both forwards and backwards, a given path in order to avoid collisions

Following individual testing of components, these pieces were integrated gradually in order to identify compatibility problems.

### Initial Setup

In order to develop the most effective autonomous solution to the challenge, the initial setup included identifying the challenge’s initial parameters and developing simple solutions to the previously outlined basic subtasks.  Regarding known initial conditions, The Labyrinth Final Challenge had the following parameters: 
1. The exit would be located at any point outside a circle with known radius centered at the starting location.
2. The maze would be constructed with opaque blocks with matte walls
3. The maze would be single-leveled, multicursal, and not simply connected
4. The resulting map would be unknown in advance

To quickly implement a SLAM system on the RACECAR, we chose to implement a version of Google Cartographer in ROS.  This allowed us to quickly create a two-dimensional map of the environment using the onboard Velodyne LIDAR, inertial measurement unit (IMU), and VESC controller odometry data. 

Drawing from our solutions to previous laboratory assignments, the path planning algorithm selected was a modified Rapidly-exploring Random Tree algorithm (RRT).  RRT enabled the car to efficiently search nonconvex, high-dimensional spaces by randomly building a space-filling tree.  Dubins curves applied constraints on the trajectory curvatures limited by steering angle and AckermanDrive model.

### Technical Approach

*SLAM*

Since Google Cartographer in ROS was used as the SLAM algorithm for the final challenge, little technical time and effort was dedicated to solving the SLAM problem.  However, the system provides two indispensable outputs to the rest of the system: localization and map generation. 

The car’s pose relative to obstacles, using a previously developed particle filter localization algorithm combined with Cartographer, provided an inferred pose for the RACECAR at all times.  Together with the mapping function, which provided probability occupancy information on a scale from 0 to 100 and a boolean representing mapped or unmapped regions, Cartographer provided the basis of the SLAM component of the challenge.

*Path Planning*

Without much a priori knowledge of the labyrinth’s layout, it was necessary to use a path planning algorithm that could perform in all situations, even cluttered environments.  An Rapidly-exploring Randomized Tree (RRT) search algorithm was chosen because we had experience with this algorithm and it could be adapted to handle dynamic constraints (all generated paths could be executed by the car).  It was decided that RRT would be more effective over RRT* because speed should be prioritized over path optimality to allow more time for the car to explore the labyrinth.  Several modifications to the RRT algorithm were made to improve performance

First, the labyrinth could be large in size and cluttered.  Thus, the sampling for the RRT, if performed over the entire map, might not be sufficiently dense to produce trajectories that move around obstacles.  To overcome this challenge, the RRT algorithm only samples points in the map if they are unoccupied and within a certain distance of either the start or end position (the certain distance used was 2.5 times the distance from the start to the end position, which concentrated the sampling but still allowed the algorithm to find paths around large obstacles).

Second, the map provided to the RRT algorithm may contain unknown areas.  Thus, it is best not to move backwards more than necessary.  To achieve this behavior, the cost function used to determine the nearest point in the tree to a randomly sampled point placed a penalty to adding nodes that moved backwards.  The cost function also promoted paths that took sharp turns which allowed the tree to grow more easily around tight spaces.

Given that the map of the cars environment is initially unknown and there is no given goal location, a strategy of determining regions that are unexplored for the car to travel to was used so that it may eventually escape the maze. We found regions with occupancy probability between 20-50%, eroded and dilated the map to remove noise, and found contour of the same color intensity to mark as regions of interest to explore. The car then would travel to a region that is closest so that it may eventually escape the maze.  

Lastly, the RRT algorithm must be built to reach a goal point quickly.  Fast motion planning means that the map can be explored more to identify a region outside of the escape circle.  To speed up the algorithm, there is a sampling bias such that 5% of all sampled points are the goal position.  Additionally, when the goal position is sampled, the algorithm checks if a dubins curve can be generated directly to the goal position without collision, rather than moving only a step size toward the goal.  Thus, if a trivial path exists to the goal, it can be found very quickly.

To ensure the avoidance of obstacles, a safety controller component was added to the path planning logic that initiated a replan if the car was expected to collide with a wall.  In this way, even if the initial trajectory caused an unplanned collision due to poor map quality, the system could adjust autonomously.

*Trajectory Tracking*

The path planning algorithm described above generates paths that are bidirectional and may move through highly cluttered environments.  Thus, it was necessary to have a method of controlling the car’s motion to accurately follow the path.  In past labs, a pure pursuit algorithm was used, which proved to be successful for high speed motion around a race track.  However, pure pursuit can result in the car cutting corners and can only follow a path in one direction.
To achieve accurate path following behavior, we chose to implement a Follow the Past (FTP) trajectory tracking algorithm.1  FTP requires a path made up of discrete poses at small distances apart, which can be provided by the path planning algorithm described above.  With this information, the FTP algorithm combines three estimates for a good steering angle: angle needed to intercept path at given lookahead distance, the recorded orientation of the car at that spot in the path, and the steering angle that the path planning algorithm used on a given section of the path (see figure 1).  

<p align="center">
  <img src="assets/images/lab8/follow_the_past_logic.png" img width="400" height="300">
</p>

<p align="center">
  <b> Figure 1: Simple block diagram of Follow the Past logic. </b>
</p>


### ROS Implementation 

The basic architecture of the system is as follows: 
The cartographer map is published to the cartographer_node which is subscribed to in the path planning logic. The first step in architecture is to use the generated map to get a list of interesting regions, defined as unexplored and potentially open areas. The point that takes the car out of the escape radius or allows it to explore potentially information-rich areas of the map is published to the topic point_to_explore. The RRT algorithm then plans a plath from the car’s current position in cur_pose_topic to point_to_explore. The resulting trajectory is published under traj_topic, subscribed by the Trajectory Following algorithm. As the map continues to grow while the car moves along this trajectory, there may be an obstacle in between the car and the point_to_explore. To avoid obstacles detected while the car follows a given trajectory, a Maze Listener node checks publishes a True boolean value should_replan which triggers a new path to be planned. This process continues until the car is out of the escape radius defined.  

## Experimental Evaluation

To ensure fluid component integration, each individual component of the Maze Solver was tested independently.  However, given the nature of the challenge, complete integration was required at an earlier stage in the problem solving process since the performance of some components would be impossible to test in simulation.

### Testing Procedure

*SLAM*

Evaluation of SLAM was done constantly throughout the lab by comparing the robot’s localized location to its position in the real world. The robot was localized very close to the real position and wasn’t an issue during any of the testing procedure.

To test how accurate of a map cartographer compared a map generated by cartographer of a known location where the map was given by the TAs. The generated map and the given map are shown below in figures X,Y.

*Path Planning*

This labyrinth challenge requires quickly making paths that the RACECAR can follow through cluttered environments.  Throughout the development of the RRT path planning algorithm, qualitative assessment of the trees growth and quantitative analysis of the runtime were used to test out modifications to the algorithm.  Specifically, the RRT algorithm was modified to include constrained sampling and an adapted cost function.  Performance of the algorithm was studied by running different searches in the simulated map of building 31, which includes both open space and tighter spaces.

*Trajectory Tracking*

Prior to system integration, the Follow the Past trajectory tracking algorithm was tested qualitatively and quantitatively in simulation and on the RACECAR.  For qualitative testing, the robot response to different trajectory inputs was observed for accurate behavior and steering response.  Quantitative testing included calculating off-track position error over the course of a given trajectory in order to to verify the accuracy of the algorithm.  Figure X depicts three trajectories that were used to gather position error information.  These trajectories include bidirectional motion and movement around obstacles, and were deemed representative of the trajectories that would be generated during the maze solving challenge.

<p align="center">
  <img src="assets/images/lab8/three_trajectories.PNG" img width="460" height="400">
</p>

<p align="center">
  <b> Figure 2: Three Trajectories used for Testing Tracking Performance </b>
</p>

Full System

To evaluate the performance of the system the robot was placed in a makeshift maze (shown below in figure X) and told to escape. Parameters, such as dilation of the map, were tuned in order to increase the escape time of the robot.

<p align="center">
  <img src="assets/images/lab8/makeshift_maze.PNG" img width="500" height="300">
</p>

<p align="center">
  <b> Figure 3: Makeshift Mazes used for testing</b>
</p>


### Results

*SLAM*

The goal of SLAM via Cartographer was to provide an accurate map of the maze while it is being explored.  The SLAM performance could be visually assessed by manually driving the car around an environment and checking RVIZ to verify the location of the car and the map that was being generated.  Such assessments identified aspects that hurt mapping accuracy.  Most notably, high acceleration values on the car caused jolting forward which blurred the location of the walls on the map.  To combat this, the acceleration parameter on the car was lowered to a value of 1.0.  Additionally, it was discovered that when cartographer ran as the car sat idle, the accuracy of the map improved very gradually.  Once the car moved, the map updated much better.  In Figure X, the map is compared before and after a short movement was executed.  After the motion, the car is much more certain what regions of the map are safe to traverse.  This discovery prompted us to incorporate a short forward and back motion into the start-up procedure of our maze solver.

<p align="center">
  <img src="assets/images/lab8/cartographer_startup.PNG" img width="300" height="300">
</p>

<p align="center">
  <b> Figure 4: Cartographer Map at Start Up.  The left picture is the map immediately upon starting Cartographer.  The right picture is the map after driving forward in a curve then back.  The accuracy of the Cartographer map greatly increases after stable motion of the car.</b>
</p>

*Path Planning*

The path planning algorithm was designed to quickly discover dynamically feasible paths around obstacles.   Because the functionality of the path planning algorithm was not conditional on the type of map used (known map vs Cartographer-generated map), the testing of the path planning algorithm was performed in simulation on a map of Building 31.  

Path planning was performed from the same start and end points with and without constrained sampling.  Figure X depicts the tree growth with and without a constraint placed on the sampling function.  Adding the constraint greatly increases the density of the sampling within the area around the start and end points, allowing for paths to be generated much quicker (see Table 1).

<p align="center">
  <img src="assets/images/lab8/constrained_sampling.PNG" img width="300" height="300">
</p>

<p align="center">
  <b> Figure 5: Effect of Constrained Sampling on RRT.  On the left, the sampling is constrained to be within a set distance of start and end points.  On the right, the sampling occurs over all unoccupied areas of the map.  Constrained sampling avoids unneeded computation on areas of the map that are far from the final path.
 </b>
</p>

<p align="center">
  <img src="assets/images/lab8/rrt_time.PNG" img width="200" height="200">
</p>

<p align="center">
  <b> Table 1: Time in seconds of RRT Path Planning for Different Sampling Strategies</b>
</p>

*Trajectory Tracking*

The goal of trajectory tracking was to create a controller that could very accurately follow a path even in tight corners and with switching directions.  In the Testing Procedure section, three trajectories were presented that were deemed representative of trajectories that would be generated during the challenge.  The position error over time is graphed below in Fig. X, Y Z for each of the three trajectories.  For all trajectories, the steady state position error is less than 5 cm.  It should also be noted that the trajectory is made up of poses separated at distances of 10 cm.  Thus, the actual steady state error may have been even lower (if the discretization of the trajectory, which was used to calculate position error, had been smaller).  A steady state position error of less than 5 cm was deemed acceptable because, through dilation of the map, it was possible to ensure the car would not hit obstacles during motion around the labyrinth.

<p align="center">
  <img src="assets/images/lab8/simple_forwards_error_plot.png" img width="400" height="300">
</p>

<p align="center">
  <b> Figure 6. Position Error in Simulation on Forwards Path.  The position error is high in the beginning as the car performs a tight turn
 </b>
</p>


<p align="center">
  <img src="assets/images/lab8/backwards_error_plot.png" img width="400" height="300">
</p>

<p align="center">
  <b> Figure 7. Position Error in Simulation on Bidirectional Path.  The spike in error in the middle of the trajectory occurs as the car changes direction.
 </b>
</p>


<p align="center">
  <img src="assets/images/lab8/complex_forwards_error_plot.png" img width="400" height="300">
</p>

<p align="center">
  <b> Figure 8: Position Error in Simulation on Complex Path.  This path was generated by running our path planning algorithm through an area with lots of obstacles. For this reason, the position error is highest compared to the other trajectories due to more changes in path curvature. 
 </b>
</p>

*Full System*

The full system integration took all aforementioned components and built a sensible architecture for the racecar to perform SLAM, path planning, trajectory tracking, and further iterations thereof. In real time testing, the car performed well in larger open space areas, as they allowed more flexibility in point sampling in the car’s path planning algorithm. When testing in tighter spaces with multiple narrow dead ends, the car often took longer than ~20 seconds to replan a new trajectory.  

<p align="center">
  <img src="assets/images/lab8/regions_of_interest.png" img width="400" height="300">
</p>

<p align="center">
  <b> Figure 9: Regions of Interest on Cartographer Map. The blue dots are areas that the car recognizes as being unexplored.  The regions of interest are not meant to be exact or perfect (some regions are actual part of another region), but act as a starting point for directing motion around the map.

 </b>
</p>

## Lessons Learned

This challenge marked the most complex problem that our team has had to tackle.  First, there was much less guidance on what algorithms should be used in the challenge.  Second, the challenge lasted three weeks, demanding lots of planning and management of tasks to ensure we did not get behind.  Thirdly, our implementation involved a lot of moving parts that needed to integrate well and work well individually (to avoid unpredictable bugs in the final implementation).  A complex problem meant a heavy workload on the team, but it also presented many opportunities for learning. 

The main lesson learned is to take small steps to the goal.  Our team has no shortage of excitement and interest in creating advanced algorithms or techniques to solve a problem.  While these algorithms are very interesting and might improve performance, they often take longer to fully implement than we have to implement them.  Thus, we often go into the last few days of the assignment with a very intricate algorithm that is still buggy and must be tested.  Instead, we would benefit from implementing an easier approach early on.  Then, we would be able to test early on, and add additional features if needed.  In this challenge, we faced this issue with the path planning algorithm.  We tried to implement RRT*, which generates optimal paths but is complex and slower computationally, but ran into bugs that created odd, unnatural paths.  Instead, we switched to RRT, which does not generate optimal paths but could be implemented within one or two days, and was simple enough to find and remove the bugs quickly.  

Another important lesson from this challenge is to prioritize time spent testing the system in a real environment.  This challenge, at the suggestion of the TA’s, we spent a lot of time testing our code on the RACECAR.  In doing so, we were more aware of issues that would come up only in the physical system, such as the behavior of Cartographer with a high car acceleration.  Additionally, it meant that we always had a test bed that well represented the final challenge, without having to spend too much time fabricating a simulated test environment. We also did not thoroughly anticipate the many different environments the car could be placed in early on in our testing process which caused some difficulties. 

Throughout of our implementation we found that having visualizations of the paths generated by RRT, clear visuals of the escape radius, and points plotted on Rviz to mark interesting regions to explore (as well as the chosen intermediate goal point) to be a helpful tool when debugging and understanding the problems and behavior of our system. 

### Technical Conclusions

When demonstrating the car’s real time performance with the other racecar competitors, we noticed a difference in complexity level between our approaches. Most competitors took a variation of a prior “wall follower” implementation (see Lab 3) with an added safety controller to handle tight spaces as their approach to escaping the maze. Albeit simple, this approach allowed the competing racecars to move quickly, and randomly explore more areas of the maze. Due to tighter spaces in the actual labyrinth setup, our car often took more time planning out trajectories. Although it did not face the problem of revisiting areas of the maze like its competitors, it took more time escaping dead ends than preferred. 

Having successfully tested the performance of racecar early in the final challenge weeks, a code freeze deadline should have been enforced so as to maintain working code and remove redundant parameter additions or tuning. The car ultimately performed better using old source code, used when videos of a functional maze solver were recorded. Had fewer parameters been tuned and more time spent on the baseline functionality of the integral maze solver components, the workflow would have been more curated and efficient. 

In summary, a simpler approach to path planning and trajectory tracking might have sufficed in completing the challenge. Nonetheless, we enjoyed implementing relatively more complex algorithms, putting effort in creating a robust, safe, reliable robotic system with this sort of algorithmic baseline, and watching the car maneuver itself out of unknown, more structured 

### CI Conclusions

[Lucas] The biggest take away from this lab is how much more cohesive/efficient a team can become after a semester of working together. I really enjoyed noticing the small ways we got better at communicating (more messages, a more standard way we wrote code, etc.) and I especially loved how much more fun we were having together and joking. All the time together meant we understood each other's humor and the joint feeling of wanting to be done helped us bond. I was also proud of how we pulled through on challenge day. Our system was not working and we collectively spent another 3-4 hours working through bugs, reverting old code, and not giving up on our approach and it finally worked really well.

[Marwa] I think we really worked well together for the final challenge and I really appreciated and enjoyed seeing how close we had become as a team at the end of it. Although we had opted for a more complicated technical approach in the beginning, the team pulled through to make a simpler approach work, tested thoroughly together and did not give up on demo day to demonstrate that our system indeed worked. 

## Future Work

Future work would include having a more specific way to choose the “interesting regions to explore” such that the car is more selected in which region it chooses as its intermediate goal point. Instead of our current model, we could define our regions to have an occupancy probability between 20-25% but also be defined as having a wall at either side (nonpermissible regions) with open space or a permissible regions between them. We could also keep track of points/regions that the car has explored previously so as to not have it tread upon a previously explored path. This would allow our maze solver to be more efficient in its exploration of paths. 

Another point of exploration for our team could be trying to make our system robust enough to identify chairs in its path and avoid them appropriately. 
