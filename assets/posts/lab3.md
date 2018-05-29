Lab 3
=====
<p align="center">
  <img src="assets/images/rc66.jpg" width:"600" height="450"/>
</p>

## Overview and Motivations - [Sean Patrick Kelley]

The focus of this lab was introductory in nature in order to ensure the safe operation of our race car.  Specifically, our objectives comprised of initializing communication with the car, driving the car autonomously, and implementing an onboard safety controller.

To initiate communication, we accessed the onboard computer via SSH, which will be the connection method moving forward.  Establishing a connection allowed us to test the pre-configured systems on the car itself, such as the motor controller by manually driving the car, and the Velodyne LiDAR by visualizing output scan data with Rviz.  

Taking a step further, the car was driven autonomously using a wall-following algorithm and Velodyne sensor data.  This code was initially developed in Lab 2 using a simulated car and virtual environment, in which the code passed tests varying initial orientation and distance.  Testing the algorithm with the actual car ensured our ability to successfully implement an autonomous controller and offered an introduction to debugging with hardware.

The final step to ensuring the safe operation of the vehicle was developing a safety controller.  Given the fragile nature of the car’s components, an onboard safety code was implemented in order to avoid collisions with walls, people, or other obstacles by stopping the car before impact.

## Proposed Approach - [Sean Patrick Kelley]

### Initial Setup - [Marwa Abdulhai]

The initial setup of this lab consisted of familiarization tasks such as connecting to the router, managing the batteries, and accessing the car through SSH. With this initial setup, we were able to manually drive the car with a remote controller and visualize the sensor data point cloud with Rviz. 


<p align="center">
  <img width="460" height="300" src="https://media.giphy.com/media/B1p5cQ03uqZ8BsugEj/giphy-downsized-large.gif">
</p>
<p align="center">
  <b> Car being manually driven</b>
</p>


<p align="center">
  <img width="460" height="300" src="https://media.giphy.com/media/1jX6AwzGYLux1q1zrd/giphy.gif">
</p>
<p align="center">
  <b> Car's laser scan result on Rviz</b>
</p>

<!-- ![66 manually driven](https://media.giphy.com/media/B1p5cQ03uqZ8BsugEj/giphy-downsized-large.gif)
*Car being manually driven*
![66's Laser Scan results](https://media.giphy.com/media/1jX6AwzGYLux1q1zrd/giphy.gif)
*Car's laser scan result on Rviz* -->
<!-- 
<center>[![66 manually driven](https://img.youtube.com/vi/m_w1h7MpIDw/0.jpg)](https://youtu.be/m_w1h7MpIDw)</center>
<center>[![66's Laser Scan results](https://img.youtube.com/vi/RyTECjRFK1o/0.jpg)](https://www.youtube.com/watch?v=RyTECjRFK1o)</center>
 -->
### Technical Approach - [David Klee, Marwa Abdulhai, Rose Wang & Lucas Novak]

This lab presented two technical challenges: demonstrating the capability to follow a wall and developing a safety controller for the car that prevents forward collisions.  The first challenge was previously approached by all team members individually during simulation testing in Lab 2.  In order to achieve wall following in simulation, the range data from the laser scan is first filtered so only the field of view looking toward the wall side is considered. Next, the filtered range data is processed using a linear regression function, numpy.linalg.lstsq, to determine the equation of the line that describes the wall to follow.  With this equation, which exists in the reference frame of the scanner, the distance from the car to the wall can be calculated.  

A PD controller is used to achieve a desired distance to the wall while maintaining a set velocity. The proportional controller steering angle is the result of a gain, KP, multiplied by the difference between the desired and actual distances:

$$\theta\_{P,steer} = K\_p(distance\_{actual} - distance\_{desired})$$

where KP is a constant and the desired distance is a constant. 

The derivative controller is slightly more complicated; the resulting steering angle is the result of a gain, KD, multiplied by the rate of change of the car with regards to the desired distance line:

$$\theta\_{D,steer} = K\_D * (\frac{d (distance\_{actual})}{dt}-   \frac{d (distance\_{desired})}{dt})$$

where KD is a constant and the change in desired distance over time is 0.0.

In order to determine the actual distance, we use the formula to calculate distance from a point to a line:
$$distance\_{actual} =\frac{ |ax\_0 + by\_0 + c|}{\sqrt{a^2 + b^2}}$$
Where the line is in the form $ax+by+c=0$ and the point is $(x\_0,y\_0)$. In our case the car is always on the origin so $(x\_0,y\_0) = (0,0)$.

In order to determine $\frac{d distance\_{actual}}{dt}$ we used the approximations:
$$\frac{d (distance\_{actual})}{dt} \approx V*sin(\theta\_{wall}) \approx V*\sin(\arctan(m))$$
Where $V$ is the velocity of the car in m/sec and $m=\frac{-a}{b}$ is the slope of the wall in the form $ax+by+c=0$.

Moving from simulation to the physical system involves additional steps.  This includes additional filtering on the range data from the scan, since the physical system introduces errors caused by reflective surfaces or moving obstacles like humans.  To reduce the effect of this error, range data was processed based on percentile range, to remove outliers.

The second challenge was to create a safety controller that would prevent the car from hitting obstacles during testing.  A minimalist approach was taken with the philosophy that an overly complex safety mechanism might interfere with other motion control commands coming from other algorithms being run.  Additional features could be added as the speed of the car increases and the obstacles it encounters become more specific.

<!-- [Pictures] -->

### ROS Implementation - [Lucas Novak]

To implement this logic, we created a ROS network consisting of two main nodes: wall follower and safety controller. The wall follower node subscribes to a topic containing laser scan data, filters the data, applies a linear regression to estimate the wall, and publishes the calculated steering angle from the PD controller in an ackerman_msg. The safety node subscribes to the laser scan data and uses it to predict forward collisions with nearby objects.  If a collision is predicted, the node publishes an updated velocity of zero at a lower level than the wall follower node.

The purpose of the wall follower node is to use laser scan data to determine the location of a wall in order to apply the formulas for PD control, described previously.  First, we adjust for the robot’s scanner offset, which we determined to be about 45 degrees. Next, we filter the laser scan data.  We only consider data from the side our robot wants to follow, or within 12.5% of total points, or 6.25% of points from the opposite side. We chose to ignore all other points, as the robot only needs to follow a wall on one side.  However we found, through experimentation, that including some points from the opposite side is beneficial for smoother turns.  We then filter the data again, this time only considering data within 2.5-25% of the maximum distance. The purpose of this filter is to reduce sensor noise that would cause points to be too close or too far, with the exact percentages chosen based on experimentation. After cleaning the data we converted the points into cartesian coordinates and applied a numpy linear regression algorithm (numpy.linalg.lstsq) to find the equation of the wall relative to the robot. With this line, we then calculate both the minimum distance to the wall and the slope of the wall, which we use as inputs to our PD controller equations.  This results in a steering angle that maintains a desired distance from the wall, publishing the calculated steering angle and the given velocity in the form of a AckermannDriveStamped message.

The safety node uses laser scan data to ensure the robot does not collide with obstructing objects, such as humans or walls.  Similar to the wall follower, we first adjust the laser scan data for the hardware offset through the minimum and maximum value parameters for an angle. Next, we determine a left and right index in order to remove all points outside the 30 degree wedge in front of the robot. We then filter out all laser scan points that are farther than 0.6 meters from the robot, which was chosen based on testing velocity and experimental performance.  In the case there are more than ten sensor points within the filtered region, the velocity of the car is set to zero in order to avoid collision. The node then publishes the updated velocity at a lower level than the wall follower node so that it may override wall follower commands. 


<p align="center">
  <img width="460" height="300" src="https://media.giphy.com/media/1ymH1eLFCEyne2gOxT/giphy.gif">
</p>
<p align="center">
  <b> Car without safety controller crashing into wall </b>
</p>

<!-- ![Why 66 needs a safety controller, wall](https://media.giphy.com/media/1ymH1eLFCEyne2gOxT/giphy.gif)
*Car without safety controller crashing into wall* -->

<p align="center">
  <img width="460" height="300" src="https://media.giphy.com/media/X6RLrXcOahXIxn5mcC/giphy.gif">
</p>
<p align="center">
  <b> Car without safety controller crashing on the ramp</b>
</p>
<!-- ![Why 66 needs a safety controller, ramp](https://media.giphy.com/media/X6RLrXcOahXIxn5mcC/giphy.gif)
*Car without safety controller crashing on the ramp</b>* -->

<!-- 
<center>[![Why 66 needs a safety controller, wall](https://img.youtube.com/vi/eBvayYkDTLk/0.jpg)](https://www.youtube.com/watch?v=eBvayYkDTLk)</center>
<center>[![Why 66 needs a safety controller, ramp](https://img.youtube.com/vi/A3U8M48aIPk/0.jpg)](https://www.youtube.com/watch?v=A3U8M48aIPk)</center> -->

## Experimental Evaluation - [Marwa Abdulhai and Rose Wang]

In order to evaluate the performance of our wall follower and safety controller algorithms, we ran a series of tests to observe their results in autonomous mode. 

### Testing Procedure - [Marwa Abdulhai and Rose Wang]

We tested our initial wall follower code on the car to compare its simulation performance to reality.  As shown in the simulation, we found that the car was able to move in a relatively straight line and maintain an average desired distance from the wall. 

However, while it was able to turn around corners, it often moved unsteadily from side to side and its movement seemed slightly delayed. When an object was detected in its forward path, the car swerved into the wall instead of turning away.  After observing this behavior in several trials, we determined the center was not at zero degrees from the robots perspective but was at an angle of sixty degrees due to the sensor data. We noted that the data may also need more filtering to remove high & low values based on experimentation and we thus decided to incorporate a low pass filter.  A post on piazza also confirmed much of the odd behavior of the car in the real world, namely that since the Velodyne rotates slowly in default mode, it returns a scan as two messages, each containing one half with real data and the other half as infinity values. Since processing these packets as pairs allowed for much more accuracy in the wall follower implementation, we attempted to incorporate these changes but did not have enough time to have it fully integrated and functional with our wall follower implementation. However, the car did fairly well with the changes that we were able to make, namly offsetting the laser scan data by approximately 45 degrees for our robot and adding a low pass filter. We were able to test this behavior on a ramp that led into a sharp turn, where the car did well and turned confidently around the corner. 

We also wanted to ensure that above all else, our safety controller was able to handle the basic collision situations and override control of the original wall follower algorithm. We first implemented a basic controller that allows the car to drive straight and halt if its front view was blocked by an obstacle about a meter in front of it. With this rudimentary version, we wanted to test out whether out refined laser scan as described above worked as intended and aimed to ensure our car does not run into obstacles. Our car was successfully able to stop when a human walked immediately in front of it or when it made such a sharp turn that would have caused it to hit against a wall. For the instance where an object such as a cone had been kept in its path for an extended period of time however, the car swerved around it instead. 


<p align="center">
  <img width="460" height="300" src="https://media.giphy.com/media/9xwMMbhE31cNO2TiZY/giphy-downsized-large.gif">
</p>
<p align="center">
  <b> Car autonomously following the wall</b>
</p>
<!-- 
![66 autonomously following walls](https://media.giphy.com/media/9xwMMbhE31cNO2TiZY/giphy-downsized-large.gif)
*Car autonomously following the wall* -->


<p align="center">
  <img width="460" height="300" src="https://media.giphy.com/media/9Pk9HxQSs54qwo7rn2/giphy.gif">
</p>
<p align="center">
  <b> Car autonomously avoiding persistent obstacles</b>
</p>
<!-- ![66 avoids cone](https://media.giphy.com/media/9Pk9HxQSs54qwo7rn2/giphy.gif)
*Car autonomously avoiding persistent obstacles* -->

<p align="center">
  <img width="460" height="300" src="https://media.giphy.com/media/vxqyUVpnhn2Nx48BpH/giphy-downsized-large.gif">
</p>
<p align="center">
  <b> Car stopping in front of immediate obstacles</b>
</p>

<!-- 
![66's Safety Controller 1](https://media.giphy.com/media/vxqyUVpnhn2Nx48BpH/giphy-downsized-large.gif)
*Car stopping in front of immediate obstacles* -->


<p align="center">
  <img width="460" height="300" src="https://media.giphy.com/media/2sXMMk8cIvkylvUdea/giphy.gif">
</p>
<p align="center">
  <b> Car stopping in front of immediate obstacles</b>
</p>

<!-- ![66's Safety Controller 2](https://media.giphy.com/media/2sXMMk8cIvkylvUdea/giphy.gif)
*Car stopping in front of immediate obstacles* -->

<!-- 
<center>[![66 autonomously following walls](https://img.youtube.com/vi/rVfJDjEdGBU/0.jpg)](https://www.youtube.com/watch?v=rVfJDjEdGBU)</center>
<center>[![66 avoids cone](https://img.youtube.com/vi/8vj9ELIQr-s/0.jpg)](https://www.youtube.com/watch?v=8vj9ELIQr-s)</center>
<center>[![66's Safety Controller 1](https://img.youtube.com/vi/R_--p2hgbvA/0.jpg)](https://www.youtube.com/watch?v=R_--p2hgbvA)</center>
<center>[![66's Safety Controller 2](https://img.youtube.com/vi/5Fktq4LfZBI/0.jpg)](https://www.youtube.com/watch?v=5Fktq4LfZBI)</center> -->

### Results - [Austin Floyd]

The primary result of this lab was the successful implementation of the wall follower ROS package required to make the car run autonomously. After importing and modifying the wall follower, our car was capable of moving alongside a wall in a (roughly) straight line. It is also, in most circumstances, able to navigate around various obstructions, including traffic cones, stationary bicycles, and human feet – provided that those obstructions have been present for some amount of time before the car’s approach. Some of our tests were inconsistent under certain conditions – for example, it would crash in narrow passages (such as the ramp in the Stata basement) even after having successfully navigated the same path beforehand. We mostly attributed this phenomenon to the car’s diminishing battery charge at the time, but this does not negate the fact that it occured.

Additionally, we were able to implement a robust safety controller capable of fully stopping the car without any required human action. This ensures that the car, and its expensive components, can avoid potentially damaging collisions. We were able to find a balance between being confident that the car could avoid a crash but still be able to complete maneuvers. At first, the car would stop for obstacles 1 meter away, which made it difficult to usefully navigate without the car stopping itself. After experimentally manipulating the numbers, we decided to base our safety controller on observed obstructions within 0.6 meters and a ±30º sweeping angle of the car’s sensor. Our car can completely stop moving when a traffic cone, for example, is placed directly in front of it. The same can be said for the car’s reaction to a person walking in front of it. Importantly, when the aforementioned traffic cone is removed, or the person standing in front of it moves away, the car continues moving autonomously using the wall follower algorithm.


## Lessons Learned - [Rose Wang and Austin Floyd]

### Version control: 
Since this was our first week working as a team, we’ve realized the importance of creating a workflow that both engages everyone in relevant tasks and coordinates our responsibilities. We created different ros packages in order to abstract the tasks and tools (i.e. wall following, safety control, laser scan shift). In the process of doing so, we created multiple files for the same task and lost track of the most recent file versions since we worked on them either over our own laptops, on the racecar platform, or in the VM. Moving forward, we will put more effort in keeping directories clean of redundant files and push our work onto Github.

### Lab instructions: 
After repeating experiments with our safety control feature, we learned that we should be more attentive to the useful tips provided in the lab. Had we read the lab with more thoroughness, we would have had a better understanding of the racecar message pipeline and saved ourselves a lot of time on improving the rudimentary features.

### CI prioritization: 
We began working on our Lab Report and Lab Briefing before we had finished implementing our safety controller. This proved useful as our attempt to finish the lab carried late into Sunday evening. We saw the value in using the report as an evolving document, and we will continue to do so in future lab assignments.


### Technical Conclusions - [Marwa Abdulhai & Rose Wang]

There were a lot of small yet important observations and conclusions that were made during the lab. Firstly, testing combinations of changes to our wall follower and safety controller did not work as smoothly as hoped. The car often ended up halting in mid-motion, and later crashing into walls in arbitrary situations, be it in a narrow or wide environment. Our testing strategy thus later evolved to incrementally apply changes to our wall follower & safety modules respectively and keep our code as simple and easy to understand as possible. This helped us move smoothly from adjusting our wall follower to have smoother turns to eventually have a simple yet functional safety controller. Although we were not able to fully integrate our rudimentary implementation of removing null values from the laser scan data, we learnt an important lesson that straightforward ideas often were the ones that became much easier to debug in real-time on the car. 

Through trial and error, we also became familiar with the mux structure provided in original [lab instructions](https://github.mit.edu/2018-RSS/lab3_wall_follower_real) and understood its importance. We originally tried to override the Ackermann Drive message published by our wall follower code by modifying the low level ackermann mux rather than the low level one. After multiple iterations of our car “almost” crashing, we realized our mistake and fixed our safety publisher topic to be accessing the high level ackermann mux. 

### CI Conclusions

1. Austin: During the first team meeting, our team decided on our preferred modes of communication. We chose Facebook Messenger for coordinating meeting times and ideas discussion, Github for sharing code, Dropbox for storage of photos and videos, and Google Docs/Slides for Lab Report and Briefing collaboration. These proved useful; there was communication amongst team members each day over the week this lab was assigned. We met as an entire six-member team several times, as well as in smaller groups to write code, debug, and test on the car. Each team member made a genuine effort to contribute to the lab both during the allocated lab time and after hours. Some problems that we identified included less-than-ideal code management (multiple versions of the safety controller, for example, were circulating without being properly pushed to our Github repository). Additionally, another project-managing platform might be a better choice for coordinating activities, due to the fact that important information could easily be ‘lost’ in Messenger.

2. Lucas: Early on it was helpful to have a team that set up a communication channel and used it consistently. It was also that everyone in the group volunteered whatever free time they had to help make the project flow easier rather than everyone trying to get out of work. During the lab I realized that there are still things we can do to be more effective and efficient with our communication. First we need to push our robot code out more often as we really only did that at the very end and it meant it was hard for people to look at the robot’s code. We may also want to find a better way of communicating what work we have done rather than just on Facebook messenger as it became hard to find specific details in the chat. Finally while working there were times where my teammates and I all tried working on similar problems or code at similar times. This made it annoying to run to test the code and annoying for everyone to attempt to work on the code. I found making temporary copies of code and giving one person long access to the robot for debugging to be good solution for this problem and we will definitely do more of this in future labs. We could also think about setting up a team contract but as of now I think we worked well enough and are on the same page to the point where I don’t know if it is 100% necessary.

3. David: It was very helpful to set up modes of communication early on, for deciding when to get together to work and for sharing code or text that had been written.  There was also a high energy level within all members of the team, with members actively seeking out time to work on the lab whenever possible.  This was necessary to overcome the issues that occurred as a result of bugs in the code and unfamiliarity with the physical car.  As the semester progresses, it will be important to keep up this energy.  To achieve this, it will be important to keep everyone engaged and try to avoid overly stressful scenarios, which might burn out the team or turn off some member’s interest.  Moving forward, I think we can be even more productive if we articulate and record, perhaps even on the first day of lab, what tasks need to be completed to finish the lab.  This way everyone will be on the same page with the workflow and we can even assign duties or let members choose which problems they want to work on.

4. Sean: Communication between team members seems to have been our greatest strength thus far: everyone is attentive to the selected messaging platform (Facebook) and responses are always prompt.  From this, everyone in the group is aware of what everyone else is working on, and thus work is completed faster and more efficiently with minimal confusion.  Furthermore, every team member actively contributed and addressed open issues with regards to the assignment, taking initiative to work on tasks in parallel as opposed to waiting for team members to finish. However, there are several areas of improvement with regards to effective team work.  Working with GitHub, including the process of updating code and sharing progress regularly, could be drastically improved.  Doing so would allow multiple team members to work on the same base code simultaneously, which may help solve logic issues faster.  Since this shortcoming was most likely a result of our collective unfamiliarity with GitHub, this problem should be easy to address with more experience.  Also, while Facebook Messenger was adaquette, other professional messaging applications, such as Slack, might be a better choice for communication given its improved organizational tools.

5. Marwa: Having a good mode of communication from the very beginning (Facebook Messenger) really helped our team coordinate and eventually meet the guidelines of this lab. All team members were very enthusiastic and willing to dedicate their time to work on the various tasks and give their best foot forward. Although we were very organized when it came to collaborating on the lab report (via google docs) or storing footage of videos/pictures (via dropbox), our team was a little uncoordinated when it came to pushing code to the repository. This may probably be due to the fact that many of us were still learning how to push our files from the racecar and debug our code on ROS, which we will hopefully become more aware of and better as a team over time. Moving forward,I look forward to continue working collaboratively as a team!

6. Rose: At our first meeting we decided on how we wanted to communicate and store our materials. It was really helpful to know how we wanted to coordinate for the project early on in the week. Since it was our first week working as a team, there were some hiccups involving work coordination (e.g. simultaneously coding for different tasks) that we hope to avoid in the future, but overall it was a positive learning experience for us all. I think as a whole, we worked well together, were communicative, and made sure to stay on top of tasks. I’m sure that over the course of this semester we’ll all improve on using Git and ROS.

## (Optional) Future Work - [Rose Wang]

In the future we hope to create a more sophisticated safety control that makes use of the [bicycle model](http://code.eng.buffalo.edu/dat/sites/model/bicycle.html), and takes more factors, such as the momentum, of the car into consideration. When ideating possible approaches to the safety controller, we also wanted to use the car’s destination as a way to optimize its path through and around obstacles. If put into an even tighter space than experimented, we hope that we can later allow the car to do more complicated path generations and fearlessly navigate itself. These latter steps would nevertheless play a more defining factor in later labs, as our car cannot yet do localization.





