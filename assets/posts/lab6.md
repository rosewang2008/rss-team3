Lab 6
=====

## Overview and Motivations - [Rose Wang]
The main focus for this week’s lab was localization, a very active area in the field of robotics. As a technical expansion to this lab, we also investigate the problem of mapping, particularly in the context of the Simultaneous Localization and Mapping (SLAM) problem, which is included in the lab’s appendix. Localization deals with estimating the position of a robot within a known map using the data from available sensors. Localization is one of the most important elements in autonomous, mobile robotics. It provides information on two key physical properties of the robot: the robot’s current position and its current orientation. Position and orientation in space are essential for modern state-of-the-art mobile robotic systems to carry out a broad range of tasks reliably. It would provide our robot with more autonomy in navigating itself in any given environment.  

In the following lab report, we first talk about the general problem of localization and elaborate on our technical approach to the problem. We later demonstrate our implementation of the Monte Carlo Localization algorithm (MCL) and evaluate how it performs in the RACECAR simulation, RViz and the real car platform in the basement of the Stata Center. As mentioned, the end of the report also includes additional extensions on mapping and SLAM.


## Proposed Approach
Our implementation of the Monte Carlo Localization algorithm is broken into three main components: the sensor model, the motion model and particle resampling. The **Initial Setup** outlines the required sensors and data we collected from the RACECAR to implement the algorithm. The **technical approach** describes MCL, then clarifies what the components of the MCL algorithm are and the role they play in localization. Finally, the **ROS implementation** illustrates how we combined the MCL components with the RACECAR platform.


### Initial Setup - [Rose Wang, Marwa Abdulhai] 
For the Monte Carlo Localization algorithm, our setup consisted of having a known map of navigation, odometry data and laser scan data. Our map provides us a defined area of space within which we’re located. The odometry data gives our motion model the information on our relative position and orientation from the given map origin. The data specifically defines our horizontal position, vertical position and orientation respectively as: 

$$(x, y, \theta)$$

Finally, the laser scan data provides our sensor model the measured distances from nearby obstacles. These distances are needed to cross-check and update our estimated location states. The RangeLibC library created by Corey Walsh was also used to optimize implementations of several 2D ray casting methods. The ROS implementation section of the report further explores its use. 

As described more specifically in the following sections, our implementation of a particle filter for MCL determines the location and rotation (more generally, the pose) of the RACECAR within the provided map using real time dometry and laser scan data. 

### Technical Approach - [Rose Wang, Marwa Abdulhai]

Under the Technical Approach, we first clarify the terms and mathematical notation we will use throughout technical report, before moving onto a higher-level description of what the Monte Carlo Localization algorithm does. Then, in more depth and technical detail, we explain the aforementioned three steps of this algorithmic process: motion model, sensor model, and particle resampling. 

#### Terms and Mathematical Notation
* Pose refers to both the location and the rotation of a given object in 2D space. 
* State and pose will be used interchangeably.
* Global localization refers to the problem of determining the current state of the system when the initial state is not known.
* Position tracking refers to the problem of tracking changes in your system state when the initial state is known.
* Location, respectively in the horizontal and vertical direction, will be expressed as $$p\_{x},p_y$$
* r refers to the recording range measurement
* d refers to the ground truth range found via ray casting
* Orientation will be expressed as  $$\theta$$
* State at time t will be expressed as the random variable, $$x\_t = \{ p\_{x,t}, p\_{y,t}, \theta\_{t} \}$$
* Action at time t will be expressed as $$ a\_t = \{ dp\_x, dp\_y, d \theta\_t\}$$
* A measured range (from our LiDAR readings) in a direction i  will be expressed as $$ r_i$$
* Observations at time t will be expressed as $$ o\_t = \{r\_0, r\_1,..., r\_N\}$$
* Belief in the state distribution at time t refers to the probability distribution over all possible states. It will be expressed as $$Bel(x_t)$$
* The probability that the robot is at pose x at time t will be expressed as $$ Bel(x_t =x)$$
* Time 0 will refer to the start time. Time T will refer to the most recent time. Time  will be expressed as 
$$ 0 \leq t \leq T$$

### Overview of MCL 

The Monte Carlo Localization algorithm utilizes a particle filter to estimating the belief distribution of possible states, each with their own weight, for our RACECAR within a dynamic environment. Given a map of the environment, it estimates the position and orientation of the robot as it moves and observes the environment. The objective is to see the particles converge towards the actual position of the robot. 

Specific to our implementation, we had two cases for setting up localization: the first of which is the  global localization problem, and second being the position tracking problem. For either case, localization occurs as the robot moves around its environment and keeps track of its motion as well as its observations. The motion model takes in an action (odometry) and the particle distribution to then output updated particle states. The sensor model takes in its observations (laser scan) to update the likelihood of a particle being in a given state. The likelihood is then updated at the end of MCL, in order to better reflect the particle distribution at time T. 

**Motion model**:  The motion model takes in the old belief distribution of particles and the last odometry data, and returns an updated pose with the odometry applied to the old distribution. At each update of the motion model, we apply a rotation matrix R onto the the positions of the particles. The degree of rotation is determined by each particle’s old orientation. R is described as follows

R = \begin{bmatrix} 
    \cos \theta & -\sin \theta \\ 
    \sin \theta & \cos \theta 
\end{bmatrix}

Since we do not have perfect odometry data, we also include random perturbations of a normal distribution to the resulting value to represent the noise in our measurements and our measurement uncertainty. Our added noise is also scaled by noise coefficients a where $$ 0 < a << 1$$

As mentioned, the action we apply onto the old particle distribution comes from odometry which acts accordingly to the geometric Ackermann model. The wheel odometry arises from the dead-reckoning integration of the motor and steering commands, and, under normal operating conditions (i.e. no extreme changes to the car’s pose due to external, environmental factor), acts less noisily than the inertial measurement unit (IMU). 

**Sensor model**: The sensor model is used to assign likelihood weights to each particle to determine the most likely location of the robot given the location of the particles and their distances relative to the wall. This allows good hypotheses to have better likelihood of being sampled, and visa versa for bad hypotheses. 

As the robot moves in an environment, it makes observations using its LiDAR. The sensor model allows the robot to updates it particle distribution more accurately to better reflect where it is in the known map. The updates are based on a recursive Bayesian estimation, which evaluates how well the actually sensed data correlates to the predicted state. 

**Precomputing sensor model**: In the sensor model, each particle’s position is updated at every iteration. To reduce computation during run-time, the sensor model is precomputed by evaluating it over a discretized grid of r and d, where r is the recording range measurement and d is the ground truth range found via ray casting. Precomputing this table allows the computation of the function to a table lookup and also provides the opportunity to numerically normalize the probability function. 

We used four cases in determining the probability of making observation o, given state x, or similarly expressed as 

$$P(o | x\_t^i) = \Pi\_j p(o | x\_t^i) \forall j \in \text{subsampled LiDAR angles}$$

This table combines the probability of four different cases:

**Case 1: Detecting a known obstacle in the map**

The probability of detecting a known obstacle in the map is represented as a Gaussian distribution around the ground truth distance between the hypothesis pose and the known map obstacle. If the measured distance d is equal to the expected distance e, then the resulting probability of the object being at that location is large. This is represented by peaked Gaussian showing that the sensor model contains a value that we expect.  

$$p(r|d) = \frac{1/\sigma \sqrt{2 \pi}} \exp(-\frac{(r-d)^2}{2 \sigma^2})$$

**Case 2: Incurring a short measurement (due to e.g. unknown obstacles)**
The probability of incurring a short measurement due to unknown obstacles is represented by a downward sloping line as the ray gets further away from the robot so as to not allow it to affect the values in the table drastically. 

**Case 3: Incurring a missed measurement (due to e.g. reflected LiDAR beam)**
The probability of having a missed measurement due to a liDAR beam hitting a wall is given a large value and is represented by a large spike at the maximal range value in the graph below, so that reflected measurements do not significantly discount particle weights. 

**Case 4: Incurring a random measurement (due to unexpected environmental catastrophes)**
This is represented by a small uniform value to account for unforeseen effects.

Putting the cases together, our probability distribution appears as a combination of a downward sloping line, a Gaussian, a small uniform value, and a spike. 

<p align="center">
  <img src="assets/images/lab5/3d-precompute.png" img width="400" height="300">
</p>

<p align="center">
  <b> Figure 1: Fully precomputed sensor model table. Distances are measured in px and are along the x and y axes. The z axis illustrates the probability of seeing “measured distance” given the ground truth value.  </b>
</p>

<p align="center">
  <img src="assets/images/lab5/sideview-precompute.png" img width="400" height="300">
</p>

<p align="center">
  <b> Figure 2: This is a probability distribution of measuring various distances given that computed distance is 135px.  Left slope is probability  of short measurement. Peak around 135px is probability of true measurement. Right peak is the probability erroneous max range measurement. Uniform value added (see ~190px).
 </b>
</p>

 
**Particle Resampling**:  With each call to the MCL algorithm, we randomly draw from a set of particles with respect to the probability weights computed in the last iteration T - 1. We call this set a proposal distribution. We use this subset of particles to then update our sensor model and the system’s weights.


### ROS Implementation - [Marwa Abdulhai, Austin Floyd]

Our particle filter is implemented in ROS within a single node. While this node contains over a dozen individual methods, as shown in Figure 3 below, the underlying methodology of the particle filter is fairly simple: the motion model and sensor model are called by an MCL algorithm (like that in the pseudocode from Figure 3 in the Technical Approach), which itself runs in a sequence of predicting and updating particle poses.

<p align="center">
  <img src="assets/images/lab5/ros_architecture_lab6.PNG" img width="500" height="300">
</p>

<p align="center">
  <b> Figure 3: Method architecture of particle filter node  </b>
</p>

We subscribe to the LaserScan data being received from the scan_topic, particles and fake_scan rostopics as well as the /vesc/odom topic containing the car’s current position and orientation, respectively. We publish the inferred pose to /pf/pose/odom topic to send an odometry message containing the car’s (x, y, theta) after running the MCL algorithm. 

For visualization, we publish a new /shifted_laser topic that accounts for the shift of the Velodyne laser scan data by approximately 45-60 degrees. We also subscribe to the topic /clicked_point to receive the pixel coordinates of the location that is clicked on the visualization map to declare where the car is initially.

We used RangeLibc, a C++/CUDA/Python library, in order for our particle filter to perform the computationally-expensive process of ray casting – calculating the ranges between particles and the nearest object – in real time. The library contains optimized implementations of several ray casting methods, including Bresenham’s Line and Ray Matching. Our sensor model uses the RangeLibc methods of calc_range_many() and eval_sensor_model() to perform ray casti


## Experimental Evaluation

In order to evaluate and improve the performance of our algorithm, we conducted a series of customized diagnostic tests on the particle filter, ultimately using a provided autograder script to compare the speed and accuracy of ourfinal algorithm to the ground truth.

### Testing Procedure - [Sean Kelley]

Each of the lab’s three main components, the motion model, sensor model, and Monte Carlo localization algorithm, needed to be tested individually to ensure proper performance. Rather than running the provided test script, we opted to simply run the pieces of our particle filter with rosbag recorded car odometry and laser scan data. With this data we were able to easily and quickly view results with rViz and identify errors using print statements.  The exact approach for each piece of the particle filter algorithm is explained below. 

<b> Debugging & Testing Motion Model </b> 
By observing only the contribution of the motion model to the MCL, bugs in the motion model could be identified quickly and addressed effectively in order to work toward a successful MCL in parallel with sensor model tuning.  While testing the motion model, the callback to the sensor model in the MCL was effectively removed by commenting.  

Based on the theory of the motion model without a sensor model, we expected the inferred pose cloud to fan out radially in the general direction of movement.  This would occur due to the lack of resampling and probability calculations necessary for the deletion of less likely resultant poses given an odometry change, thereby leading to the retention of unlikely trajectories.  These expectations were confirmed by observation, as shown in the rViz simulation below.

<p align="center">
  <img src="assets/images/lab5/results_jpg/points.png" img width="750" height="300">
</p>

<p align="center">
  <b> Figure 4: RViz simulation of the motion model.  As expected, due to the lack of resampling and a sensor model, the particles "fan out" over time in the general direction of movements since the least likely trajectories are retained.  </b>
</p>

<b> Debugging & Testing Sensor Model </b>
In order to debug the sensor model, the same process was followed by instead commenting out the motion model.  The expected response of the system to odometry message changes, given the lack of a motion model, would be a point cloud that remains in the same location, but which grows and shrinks in coordination with the car’s pose ‘confidence’.  When turning or in an a non-unique area of the map, the point cloud would grow to show relative uncertainty compared to locations in intersections.  

These behavioral expectations, after several iterations of editing and revising, were observed using rosbag recorded data from both physical and simulated car trajectories.  This step of the process was largely qualitative in nature, answering the question: does the behavior make sense? Once the results were as expected, the sensor model could be integrated with the motion model and the combined performance evaluated quantitatively.

<b> Combined Performance Evaluation </b>
With a final, high performing version of our algorithm achieved, we tested the accuracy and the degree to which our algorithm is deterministic using rosbag recordings from driving in the perfect odometry environment of a simulation.

<p align="center">
  <img src="assets/images/lab5/results_jpg/code.png" img width="600" height="200">
</p>

<p align="center">
  <b> Figure 5: The ROS implementation of the pose error script.  This allowed for the direct comparison between the algorithm’s inferred pose and the ground truth provided by the simulation. The error topic was echoed and a csv file of the data was collected in order to generate the resulting plots.  </b>
</p>

In order to quantify the accuracy of our particle filter, three different simulation datasets, in rosbag form, were recorded from three different simulation trajectories: up the empty hallway, down the hallway with the pillar, and down the hallway with the bikes.  In the cases of driving the car in the wide hallways, the car was driven in a sinusoidal pattern, as opposed to a straight line, to achieve the greatest potential pose error.  

A new topic, entitled position_error, was published which reported the absolute distance between the simulation ground truth odometry and the corresponding pose from the particle filter.  From this data, the cumulative position error over time could be evaluated, and the results are shown in the following section.  Each rosbag was run ten times, with the spread of resulting trajectories also shown in the graphs, in order to determine the repeatability of the algorithm.

 
### Results - [Lucas Novak, Sean Kelley]

Here we describe how we evaluate how our algorithm performs by comparing the estimated inferred pose to a ground truth in a series of simulations. Our final result, when running our code on the provided test, achieved a rate of 49.91 Hz: 24.5% faster than the “real time” goal value of 40.00 Hz. 

<b> Simulations: </b>

Note the intersection referred to is the one in the left-center of the map bisecting the main hallway.  The orientation graphs show sudden jumps from 0 to pi. These steps should be disregarded, as this was included to prevent calculating negative orientation changes.  The true error would therefore be the absolute negative distance from these greater points to pi.

<b> Simulation 1 </b>
This simulation tested the car starting at the intersection and driving until it reached the top of the hallway. The average error and its spread over ten trials is shown in Figures 6,7.  In Figure 6 we see the position error of the localized car centers around .15 meters and spikes up twice during the simulation.  Because the car is going down the hallway, it may be due to the lack of features the laser scan is detecting, as it only knows how far away from wall it is. This would lead to the car being uncertain of its exact location, and subsequently weighing many points in a line equidistant from the wall with similarly, thereby increasing error. Additionally, Figure 7 shows a small orientation error over time: either a little above 0 or a little below pi, which as discussed in the note above, is equivalent to positive and negative errors around zero. There is one spike but this is probably due to random variation. 

The variance in position data, shown by the blue shaded region in figure 6, grows significantly around the spike in position error.  This behavior is expected, since when the position error grows significantly larger than the other timesteps, the racecar is relatively unconfident in its inferred pose.  Thus the explanation for the wide range of position errors is similar to the explanation for the lack of positional accuracy: limited features in the straight hallway do not allow for precise localization due to non-uniqueness of the environment.

<p align="center">
  <img src="assets/images/lab5/results_jpg/new_all1_pos-1.jpg" img width="300" height="400"> <img src="assets/images/lab5/results_jpg/new_sim1_or-1.jpg" img width="300" height="400">
</p>

<p align="center">
  <b> Figure 6,7: Simulation 1: Mean position error and range of position error over 10 trials (left), and average orientation error for 10 trials (right). </b>
</p>

<b> Simulation 2: </b>
This simulation had the car starting at the intersection and driving past the pillar (towards the 6.141 lab). The average error and spread for this test is shown in Figures 8,9. In Figure 8 we see the localized car stays close to the actual car with fluctuation around 0.2 meters. Near the end of the simulation, the error drops significantly which could be explained by the car passing the pillar: a unique landmark that would make it easier for the localization to be correct. Again, as can be seen from Figure 9, the orientation error effectively varies between ±0.25 radians. 

The variance in errors over the 10 trials is noticeably less than the spike from trial 1, as it never exceeds ±0.10 meters.  This improved repeatability can be attributed to the increased number of distinct features available for detection in the tighter hallway with such objects as pillars and inconsistent wall geometries.

<p align="center">
  <img src="assets/images/lab5/results_jpg/new_all2_pos-1.jpg" img width="300" height="400"> <img src="assets/images/lab5/results_jpg/new_sim3_or-1.jpg" img width="300" height="400">
</p>

<p align="center">
  <b> Figure 8,9: Simulation 2: Mean position error and range of position error over 10 trials (left), and average orientation error for 10 trials (right). </b>
</p>

<b> Simulation 3: </b>
This simulation was of the car starting at the intersection and ending right after the end of the bike rack.  The average error and performance variation for the tests can be seen in Figures 10,11. In Figure 10 we see that overall the localized car stays near the actual car, averaging about 0.2 meters again, but with more variation than simulation 2. This variation is probably caused by the laser scan data being worse due to the bikes. Figure 11 again shows a low orientation error, again bounded by ±0.25 radians. Like simulation 2 the orientation error seems to fluctuate more than simulation 1 but it is still hard to see any relationship between it and position error. 

The variance in errors for trial three is unique from the previous two simulations in the sense that the variance grows gradually over time, starting at about ±0.05 and ending around ±0.15 meters.  This can be explained with knowledge of where the simulation moves.  In the beginning, the robot is near the identifiable intersection, and over time moves past the bikes to the bottom of the hallway.  Thus, this data effectively shows the relationship between unknown and unexpected wall geometries (the bikes) to increased localization uncertainty.

<p align="center">
  <img src="assets/images/lab5/results_jpg/new_all3_pos-1.jpg" img width="300" height="400"><img src="assets/images/lab5/results_jpg/new_sim3_or-1.jpg" img width="300" height="400">
</p>

<p align="center">
  <b> Figure 10,11: Simulation 3: Mean position error and range of position error over 10 trials (left), and average orientation error for 10 trials (right).. </b>
</p>

<b> All Simulations: </b>
The results of the three simulations mentioned above plotted together can be seen below on Fgures 12,13. In Figure 12 we can see that the position error for all the simulations are similar. The biggest differences come from simulation 1 having the two biggest spikes and simulation 2 ending with a higher error than simulation 1 or 3. Our idea that the error in simulation 1 is caused by the non uniqueness of the hallway still makes sense as both simulation 2 and 3 don’t have that problem and don’t have those spikes. In Figure 13 we see that the orientation error of the simulations is about the same, barring one big spike due to random variance.    

<p align="center">
  <img src="assets/images/lab5/results_jpg/new_all_pos-1.jpg" img width="300" height="400"><img src="assets/images/lab5/results_jpg/new_all_or-1.jpg" img width="300" height="400">
</p>

<p align="center">
  <b> Figures 12,13: The average error [position on left, orientation on right] of all errors where the colored area represents the values between the min and max error over all the trials.  </b>
</p>



## Lessons Learned - [David Klee]

The technical goals of this lab marked a significant increase in complexity over the work of previous labs.  The increase in complexity forced the team to develop stronger organizational and planning habits.  Additionally, we have come away with a better sense of how to most efficiently debug a complex algorithm.

The first major takeaway from this lab was to divide the team into smaller working groups where each group is delegated an ample amount of work, to improve efficiency and organization.  Building off our experiences in previous labs, we began the work by ensuring all members on the team had a good foundation in the technical aspects of particle filters, by reading through the lab handout and reviewing the textbook, Probabilistic Robotics by Thrun and Burgard.  This was important because we next split into three groups of two, each tasked with implementing one core section of the MCL algorithm.  Dividing into pairs allowed us to make progress more efficiently, since we had seen in the past that it was difficult for the whole group of six to remain productive when meeting up together.  We will likely continue with this approach in the future because it proved successful and each of the three core tasks was completed quickly.  

The second major lesson learned was to incorporate debugging into every step of development.  This is a lesson that is recurrent throughout this class.  However, at every step the debugging process can seem tedious or unnecessary so it is often overlooked.  In many ways, we have made improvements on this issue, and we tried to incrementally implement the MCL algorithm (by testing the motion and sensor models in isolation first) to catch bugs early on.  Still, we stalled for two days due to an error with one line of code, which was messing up our entire MCL algorithm.  So as we move forward, we will make sure to emphasize the importance of debugging and code reviews even for the smaller tasks or pieces of code.  One idea is to have each 2-person working group briefly review the code of another group before testing.  Furthermore, the increased focus on generating quantitative data to evaluate our algorithms, which has been stressed by the instructors, will likely aid in the debugging.  If we are continuously thinking of what metrics each piece of code needs to meet and how to test that, then we will catch bugs or poor performances early on before the complexity of the system makes identifying problems more time-consuming and difficult.  At the same time, approaching the problem with a keen sense of what performance metrics must be hit will allow us to generate more technically convincing reports. 


### Technical Conclusions - [Austin Floyd, Rose Wang]

While developing our implementation, we were careful of the following pitfalls:
1. coordinate conversions
2. vectorization with numpy
3. caching and reusing large buffers
4. concurrency problems
5. implement visualization early to help debugging

Once we had finished writing code for individual parts of the particle filter (motion model, sensor model, particle initialization/MCL), we spent several days attempting to debug by running all of these components simultaneously. This proved to be an ineffective strategy for quickly identifying bugs. After receiving a tip from a TA, we began testing components individually. We started by running only the motion model using the joystick controller; we were able to see that the simulated car did not move as intended in certain directions, and resolved the problem within half-an-hour.


### CI Conclusions 
[Marwa] Our team decided early on to split the 3 main algorithms (motion model, sensor model, and MCL) amongst 3 teams of two people each to finish things in a timely manner. However, we faced difficulty when debugging our combined implementations as our individual parts were not fully functional. Due to spring break, we also were on a time crunch to get things done. In addition to working in pairs, it may be better to get together as a group and debug together/debug our individual components better. Dividing up the tasks in finer detail may also be helpful in letting everyone have a big enough technical part. 

[David] This assignment spanned 3 weeks, two of which the team was around, and involved a much more difficult task than the previous labs.  In the past, we ran into issues with debugging and were very crunched on time the last few days to get our algorithms to work and produce the report and briefing.  We concluded that we were not operating in the most organizationally efficient way, so we decided to create 2-person working groups this time, each of which would work on a single, large task.  While in the groups, work went quite well as each member seemed to be kept busy.  However, we reconvened and tried to test all three pieces of code together too early.  Upon reconvening, we worked inefficiently, since only a few people could productively debug at a time, and people were looking for bugs in pieces code that they were not familiar with.  In the future, I think we need to continue along further while split in the groups.  Only combining the results of each group once it is demonstrated that each task is fully completed (i.e. debugged to the point that it runs independently and there is a concrete performance evaluation of the piece of code in tables or graphs).  I think we are moving in the right direction but we need to keep improving our productivity.

[Austin] This three-week lab was comprehensive and arguably more challenging than its predecessors. Our team, learning from those previous labs, decided at the start to split into three groups and assigned separate tasks to each. We did a good job of getting seemingly-functional versions of those tasks written, but the combination of those parts was somewhat chaotic and unproductive. We started off individually debugging our code on the master branch, which led to many merge conflicts. We then switched to using branches, which also became a problem when trying to recombine various functional parts from each branch. I’m still unsure what the best strategy for debugging is, but we had success in doing so on a single person’s computer.

[Sean] Since this lab represented a significant increase in difficulty level to its predecessors, we made it a point at the beginning to establish a three-team system in order to work as efficiently as possible.  More the most part, this worked quite well in the beginning, as everyone was busy with their specific task.  However, issues arose with combining our efforts and producing material for the presentation and lab report.  As others have mentioned, we convened too soon, and did not thoroughly test our individual algorithms. To address this, and the problem of our lack of data analysis materials, each subteam should work toward quantitative results instead of simply a working algorithm.  This would result in a requirement to attain evaluation data needed for the report as well as ensure the algorithm not only works, but works well enough, for team integration.

[Rose] This lab was overall a very challenging lab as it was a first dive into the fundamentals of robotics. Beyond visual perception (last week's lab), localization will enable our car to pursue comprehensive autonomous tasks. As such, our team took strong initiative at their respective parts. It was indeed difficult to debug the parts together: beyond the sensor model and the motion model, necessary debugging was needed for visualization (e.g. performing the correct transformations) for future use. That being said, although we did reconvene together early in the lab process, I believe the challenges we faced are innate properties of team projects. Nevertheless, we should still continue to improve our Git workflow.

[Lucas]The length and difficulty of this lab made coordination and working together more difficult than previous labs. We tried pairing people together and that helped a lot initially it did make us run into a bottle neck when debugging when integrating parts together. We also did a lot better working remotely over spring break. We also did a lot more to address the lack of analytical analysis. Some people made the simulations and I made the graphs. This was a little tricky because we had to communicate what the data was about but in the end it worked out well.

## Future Work - [David Klee]

For this lab, the team built an efficient MCL algorithm that provides accurate pose estimation for the racecar.  From an academic perspective, this exercise exposed us to interesting topics in the very relevant field of localization for mobile robots.  Without constraints on time, it would be interesting to see how effective the MCL algorithm would be if measurements were made using the car’s camera, via feature extraction perhaps.   More likely, our future work will build off our localization algorithm to move us toward a racecar that can move quickly around the racetrack.  For instance, now with a strong sense of the racecar’s pose, it is possible to generate trajectories that the car can follow at high speeds without concern of running into obstacles.  

## Technical Extension: Mapping - [Marwa Abdulhai, Rose Wang]

### Overview and Motivation 
This additional section of the lab report introduces mapping with assumed knowledge on localization in the context of the SLAM problem, the simultaneous localization and mapping problem. It elaborates on our proposed SLAM solution using Google Cartographer, a real-time SLAM library. The report outlines the performance of our implementation and analyzes its performance with a qualitative metric. We highlight how we would further extend this analysis with a quantitative approach and make suggestions on methods of improving our SLAM implementation and integrating it with the aforementioned particle filter algorithm, MCL. The report concludes with a comparison of the existing state-of-the-art SLAM types. 

### Theory Behind SLAM 
Simultaneous localization and mapping (SLAM) is the problem of creating a map of an unknown environment while keeping track of a robot’s location within it. This problem of learning maps while having an understanding of location is critical in mobile robotics. Models of the environment are needed to support autonomous mobility of robotic agents. 
 
Most popular algorithmic solutions addressing this problem rely on approximating the robot’s pose with the environment’s landmark features and improving the map landmark position estimates with the robot’s pose estimation. Indeed SLAM appears to be a chicken-and-egg problem. It opens up two questions: how should a robot localize itself if no prior knowledge of the physical environment is provided? And, how should a robot create a map of its surroundings, if it has no prior knowledge of its relative location? 
 
### Our Implementation 
Our solution to SLAM involves using the Cartographer, a real time SLAM library developed by Google. The following section outlines the structure of Cartographer and describes our approach to tuning the Cartographer system.

The Cartographer system is made of two parts, local SLAM and global SLAM. Local SLAM generates submaps of the environment while global SLAM matches the laser scan against submaps to find loop closure constraints to consistently tie the submaps together. 
 
Local SLAM and global SLAM are intertwined to generate the resulting map of the robot’s environment. On the local SLAM side, the submaps are generated from small localized maps from a single LiDAR scan and are represented as nodes in the a graph. The edges between nodes represent geographical constraints. This graph constrained by loop closures is solved using a Ceres solver, an open source C++ library used to solve non-linear least squares problems. We define loop closures to be recognizing a previously-visited location and updating the model’s beliefs accordingly.
On the global SLAM side, we can draw a map using this graph. We derive loop closures by identifying relationships between nearby nodes that do not have an edge between them using scan matching. Adding an edge between these nodes allows the loop to be closed the next time the map is drawn.
 
### Tuning the Parameters 
We purposefully tuned individual parameters to compare against the result produced by the original set of parameters (reference 1, appendix). This tuning approach is meant to highlight the parameter role in the overall system, and discern how its tuning would affect the result of our map. In order to ensure for consistency in testing environment and to generate interesting maps, our car remained in the same location near to corners and open corridors. A floor map including the robot’s pose (in red) is illustrated below and will be used to compare map results from our Cartographer tuning experiments. We define the floor map as a preliminary ground truth since it reflects the basic structure of the floor, however does not include existing building furniture (e.g. tables, chairs). 

<p align="center">
  <img src="assets/images/lab5/mapping/floormap-with-car.png" img width="200" height="300">
</p>

<p align="center">
  <b> Figure 19: Floor Map of Building 31, Floor 2 and car (as red rectangle) including its orientation as indicated by the red arrow. This space is used for qualitative analysis of our Cartographer implementation and will be used as a preliminary ground truth for following illustrative comparisons.  </b>
</p>

The following describes the parameters we tuned to achieve our results and explains our principle approach to tuning them. The first two parameters were observed to yield lower latency during map generation and the latter were observed to increase map definition and accuracy. We define low latency to be receiving an optimized local pose immediately after sensor input is received (within a second) with no backlog for global optimization; we define map definition and accuracy by qualitative metrics in observing deviations in resulting map from our ground truth floor plan. 

<b> 1. `TRAJECTORY_BUILDER_2D.submaps.num_range_data`: </b> This value configures the size of the submaps.  Since Local SLAM drifts over time, only loop closure can fix the drifting of the laser scan. Since size of the submap is correlated with this drift, we must reduce the submap to be small enough so that the drift inside them is below resolution to be locally correct, and large enough to be distinct for loop closure to work correctly. Taking the suggestion from Google’s Cartographer documentation, we reduced this value from 100 to 20 to lower latency. 

<b> 2. `POSE_GRAPH.optimize_every_n_nodes`:</b> This value tunes pure localization where we expect low latency of both local and global SLAM. Since the `TRAJECTORY_BUILDER.pure_localization` parameter was already set to true, we only had to decrease this value to receive more frequent values. However this results in the global SLAM being too slow, so we had to also decrease `global_sampling ratio` and `constraint_builder.sampling_ratio` to compensate for a large number of constraints. We thus reduced this value from 10 to 1 to lower latency. 

<b> 3. `Submap_publish_period_sec`: </b> This is the interval in seconds at which we publish the submap poses. Originally set to 0.3 seconds, we increased the value to 1 to yield a quickly generated map. This resulting map was able to more efficiently identify the permissibility of areas and generated a more defined map.  Free space and obstacles, such as walls, were respectively assigned a higher probability of being permissible and non-permissible. With a higher publishing rate for submap poses, the car’s map experienced less drift from the its persistent LiDAR data.

<b> 4. `Num_subdivisions_per_laser_scan`: </b> This is the number of point clouds to split each received (multi-echo) laser scan into. Subdividing a scan makes it possible to unwarp scans acquired while the scanners are moving. There is a corresponding trajectory builder option to accumulate the subdivided scans into a point cloud that will be used for scan matching.

### Our Results 
The following section qualitatively analyzes the map generated by the original Cartographer values (reference 1, appendix (#reference-1)) and the respectively tuned parameter. We opted for a qualitative metric for evaluate a successful implementation of Cartographer given the limited access we had to the car and the time needed to generate the ground truth of any environment of our choice; nevertheless, in the subsequent report section, we explore future improvements to our implementation. 

To ensure consistency in our analysis, we kept our car location static and changed the aforementioned parameters to measure the rate at which the map was generated in a span of 2 minutes and how sharp the boundaries it generated were.

Our final parameters can be found under reference 2, appendix (#reference-2) in the [appendix](https://github.mit.edu/rss2018-team3/cartographer_config/blob/master/racecar_2d.lua)

The below figures illustrate where our car was located through the experimental evaluation. The right figure includes a general outline of which edges the car has in view indicated in blue.

<p align="center">
  <img src="assets/images/lab5/mapping/floormap.png" img width="200" height="300">  <img src="assets/images/lab5/mapping/floormap-with-laser.png" img width="200" height="300">
</p>

<p align="center">
  <b> Figures 20,21
   </b>
</p>

Our base comparison is the map generated after two minutes with the original values [reference to app]. Notable features of this map is the poorly defined and heavily drifted corridor space to the front end of the car. The left figure is the original floor map, and the right figure is the map generated. 

<p align="center">
  <img src="assets/images/lab5/mapping/0.png" img width="300" height="300"> <img src="assets/images/lab5/mapping/1-a.png" img width="300" height="300">
</p>

<p align="center">
  <b> Figures 22,23: Tuning num_range_data: We found the map of the car become finer and less blurry when changing the TRAJECTORY_BUILDER_2D.submaps.num_range_data parameter from 100 to 50 as shown in the image below. We thus reduced it to 20 to remove make the map as certain about its mapping boundaries as possible.
   </b>
</p>

<p align="center">
  <img src="assets/images/lab5/mapping/0.png" img width="300" height="300"> <img src="assets/images/lab5/mapping/1-b.png" img width="300" height="300">
</p>

<p align="center">
  <b> Figures 24,25: Tuning optimize_every_n_nodes: We decreased the value of the POSE_GRAPH.optimize_every_n_nodes parameter from 10 to 1 and found the map generated to be more defined and solid in shape. 

   </b>
</p>

<p align="center">
  <img src="assets/images/lab5/mapping/0.png" img width="300" height="300"><img src="assets/images/lab5/mapping/1-c.png"img width="300" height="300">
</p>

<p align="center">
  <b> Figures 26,27: Tuning submap_publish_period_sec: The left picture shows the resulting map after two minutes with the original parameter values The right pictures shows the resulting map after two minutes with `submap_publish_period_sec` parameter tuned from 0.3 to 1. We found the map to have an increase in white area showing that it is more certain of the map it has generated. 

   </b>
</p>

<p align="center">
  <img src="assets/images/lab5/mapping/0.png" img width="300" height="300"> <img src="assets/images/lab5/mapping/1-d.png" img width="300" height="300">
</p>

<p align="center">
  <b> Figures 28,29: Tuning num_subdivisions_per_laser-scan: The picture shows the map generated after changing this parameter from its default value of 2 to 1 including all the other aforementioned changes. The tuned parameter introduced a defined map and high probability certainties about space permissibility as indicated by the shade of map color.

   </b>
</p>

Not only did we qualitatively measure map generation at a corner, but we also observed it in an open space area. Here we see the two different maps for two different parameter sets: the map generated when changing only the `submap_publish_period_sec` and `Num_subdivisions_per_laser_scan parameters`, and the map generated when changing all 4 values. We see that the latter map indicates more certainy about its map, however is less defined due to the blurriness of the map edges. 

<p align="center">
  <img src="assets/images/lab5/mapping/gmapping-open_space.PNG" img width="300" height="300"> <img src="assets/images/lab5/mapping/gmapping-open_space-change_both.PNG" img width="300" height="300">
</p>

<p align="center">
  <b> Figures 30, 31: Generating a map in open space 
   </b>
</p>

We suspected that the car's IMU state estimation introduced drift in map generation, which we confirmed by performing consecutive experiments for each of the tuned parameter tests. Our testing procedure allowing the car to remain static at its start position for two minutes, drive the car down the hallway, then allow the car to remain static at the end of the hallway for another two minutes. 

We noticed consistent behavior of the following type for all tests: maps would be generated in a more defined manner when the car remained static. Any movement introduced to the RACECAR would trigger large deviations in the generated map from the floor map. These deviations are primarily noticeable how the map would be rotation around its origin the RViz. If the car assumes a static position post-movement, these rotations would continue to perpetuate map generation and cause incorrect circular representations of the room environment. 
We also demoed drift and deviation that arose with car movement and noticed that RACECAR’s fast acceleration induced a drift in map generation


<p align="center">
  <img src="assets/images/lab5/mapping/init.png" img width="300" height="300"> <img src="assets/images/lab5/mapping/drive.png" img width="300" height="300"> <img src="assets/images/lab5/mapping/post-drive.png" img width="300" height="300">
</p>

<p align="center">
  <b> Figures 32, 33, 34: We demonstrated that car movements introduced drift in map generation for all parameter-tuning tests conducted. These figures are meant to clearly illustrate our experimental procedure for demonstrating drift in the map generation. The first figure indicates where the car assumed a static position (start position) for 2 minutes. The second figure shows that car movement down the hallway after the initial 2 minutes. The third figure shows the car's end position where we allowed map generation for 2 minutes. These figures are meant to complement the three additional RViz wfigures below.
   </b>
</p>

<p align="center">
  <img src="assets/images/lab5/mapping/[demo-drift]pre-drive.png" img width="300" height="300"> <img src="assets/images/lab5/mapping/[demo-drift]drive.png" img width="300" height="300"> <img src="assets/images/lab5/mapping/[demo-drift]post-drive-wait.png" img width="300" height="300">
</p>

<p align="center">
  <b> Figures 35, 36, 37: We demonstrated that car movements introduced drift in map generation for all parameter-tuning tests conducted. We first allowed the car to sit and generate a map in a static position; figure 35 is the resulting map after two minutes. We then drove the car down the hallway; figure 36 is the reultling map when the car passed the hallway's midpoint. Finally we allowed our car to assume static position at the end of the hallway; figure 37 is the resulting map after an additional 2 minutes. Noticeable deviations from the original floormap of the room were noted in the map created after all experiments performed. 
	</b>
</p>


### Improving Implementation & Evaluation of Success

In the future, we would like to change change our parameters such that we have a clearer looking map. Specifically, we did not decrease the global_sampling ratio and `constraint_builder.sampling_ratio` when trying to compensate for a large number of constraints after decreasing the value of the `POSE_GRAPH.optimize_every_n_nodes.` 

In addition, we would like to use the iterative closest point (ICP) algorithm to stitch consecutive point clouds in order to generate a ground truth for our environment of choice. This would enable us to do testing beyond the class-provided simulator and to generate a diverse set of experimental results in different environments. 

As an alternative way of evaluate the success of our tuned parameters, we could compare the localization of the car in our generated map using SLAM to localization on our ground truth, which can be taken to be the staff-provided map of stata basement.
 
Other aspects we would like to explore are the additional [built-in tools](https://github.com/googlecartographer/cartographer/blob/master/docs/source/evaluation.rst) provided by Cartographer for quality assurance purposes that can be used to access the SLAM result when there is no dedicated ground truth available. This would provide us with a quantitative way to evaluate our test data against the generated ground truth of any chosen location. 


### Integrating SLAM into Particle Filter

This lab section explores a possible approach to integrating the two lab topics of localization and mapping together--in other words, a discussion of a Particle Slam algorithm. Note that in our implementation of MCL, a particle is a candidate robot pose. In our implementation of Google Cartographer, a particle consists of a pose and a map. 

Our Particle SLAM would work as follows. 
For each particle, we would: 
* Move the particle. The particle’s change in pose is similar to what happens in the motion model of MCL. 
* Predict the observations. Similar to our implementation of MCL, this is calculated based off the the particle’s pose and its own individual map (rather than the global map). 
* Update the particle’s map. This map is updated based off of the particle’s individual observations and map of a previous timestep using a Bayes’ filter. This is similar to how we update the global map in MCL. 
* Weight the particle. Similar to our implementation of MCL, we evaluate the likelihood of the individual particle’s pose and map given the predicted observations and its sensor model. 
Finally, we resample the particles using weights.

As one may have noted, we would need to consider a more efficient way of storing all of the particles’ maps. One approach to achieving space-efficiency is creating a data structure such as a Quadtree. A Quadtree compresses multiple small values representing open space area (i.e. voxels) down to a single big value. In a tree representation, this signifies groups of discrete map cells are merged into one big map cell. Although this method of map storage is susceptible to less precision in our particle probabilities, it can compress maps up to 20 times their original space.

### Comparing Types of SLAM 
There are two main types of SLAM algorithms that were discussed in lecture: EKF Slam and Particle Filter. On a high level, EKF and Particle Filter take filter-based approach which updates the map model and particles’ pose recursively. It maintains the information about the environment and the states of the robot as a probability density function. Google Cartographer takes on a global optimization approach, which saves keyframes in the environment and uses bundle adjustment to estimate the motion.

* EKF SLAM uses a extended Kalman filter based approach for the simultaneous localization and mapping (SLAM) problem. The solution to the problem is represented as a Gaussian distribution over state variables. It stores the correlations between all location estimate errors so that when the robot observes a landmark and updates its location, it will also know more about the location of the other landmarks and robot pose because their errors are correlated. This was previously the standard algorithm for SLAM until FastSLAM which was able to deal with uncertainty unlike EKF. This algorithm is assumed to have a gaussian noise proportional to the controls (x, y, theta) which causes linearization to fail during uncertainty. This algorithm maintains one hypothesis of the solution unlike the particle filter approach which tracks multiple hypothesis. 

* Particle Filter SLAM: Reproduces the work of the Kalman filter in non-linear/non-gaussian environments. The particle filter algorithm uses simulation methods to generate estimates of states 

Out of the three SLAM algorithms, we believe Cartographer is the best option because of its optimization approach. The optimization approach supports loop closure more stringently than a probability approach. Loop closures are critical to generating accurate maps as they generate assertions about previously visited location. Recognizing loop closures is crucial for enhancing the robustness of SLAM algorithms, and is very relevant for addressing the global localization problem mentioned under our Localization solution. 


Appendix for Mapping
===============================================================================

## The original values provided for Google Cartographer <a id="reference-1"></a>

```
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_imu_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = false,
  use_odometry = true,
  use_nav_sat = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 2,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 4.0
TRAJECTORY_BUILDER_2D.use_imu_data = true

-- might be able to optimize these parameters
-- see: http://google-cartographer-ros.readthedocs.io/en/latest/tuning.html
-- TRAJECTORY_BUILDER_2D.submaps.num_range_data = 100
-- POSE_GRAPH.optimize_every_n_nodes = 10

return options

```


## Our final parameter values for Google Cartographer  <a id="reference-2"></a>

```
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_imu_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = false,
  use_odometry = true,
  use_nav_sat = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1.,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 1.0,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 4.0
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 20
POSE_GRAPH.optimize_every_n_nodes = 5

return options


```






