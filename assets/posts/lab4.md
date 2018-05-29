Lab 4
=====

## Overview and Motivations - [Marwa Abdulhai]

The focus of this lab was implementing various computer vision techniques in order to detect known objects and to execute various predefined tasks relative to those objects. The objectives of this lab could be split into the following 4 sub-parts respectively:
Identify a bright cone offline using Color Segmentation
Apply a 3D space transformation to determine the location of the robot relative to the cone
Implement a PD control algorithm to park the race car in front of the cone
Implement a similar control algorithm to make the racecar follow a bright orange line on the ground. 

The first part consisted of evaluating the performance of three different computer vision techniques, including SIFT RANSAC, template matching, and color segmentation, on sample test images to determine the most reliable algorithm.  To convert pixel image coordinates to real world locations we manually computed a homography matrix specific to the ZED camera, and created a function to apply a coordinate space conversion.  With the converted coordinates, we developed a controller that parks the robot in front of a cone at a given distance. Finally, we adapted these algorithms for use as a line following controller, which we used to autonomously follow a brightly colored tape line while moving at high speed. 

The expectation is that the subparts of this lab will help us with future tasks of identifying other known objects and using the camera for collision avoidance, perception, and mapping problems.

## Proposed Approach 

### Initial Setup - [Sean Kelley]

The initial setup of the lab dealt with developing the capability of using visual perception for motion planning.  Specifically, use of the Velodyne LiDAR for object detection was replaced with a Stereolabs ZED camera mounted on the front of the vehicle.  While the camera can automatically detect object depth in images, in order to demonstrate simple image based navigation, only the raw RGB image from the camera was used.  With this setup, we were able to transform image coordinates for use with motion controllers, as discussed in the following sections.   
 
### Technical Approach - [Rose Wang, Austin Floyd, Marwa Abdulhai, Lucas Novak]

Many methodologies exist for object detection in an image.  In order to determine the best algorithm to use, we implemented three strategies independently: SIFT RANSAC, template matching, and color segmentation.  

**Computer Vision Algorithms**

Here we define the three computer vision algorithms investigated in this lab to evaluate their strengths and weaknesses.

SIFT RANSAC: This algorithm uses the scale invariant feature transform (SIFT) to extract keypoints/local features in an image and compute their descriptors, a 128 element vector that describes the edges in a subregion of the image. The opencv python library was used in this case to compute SIFT on a template and test image, and find matches using RANSAC. If there are enough good matches (minimum of 75%), a bounding box is drawn around the object. SIFT can identify objects even among clutter as it is invariant to uniform scaling and orientation, however it does not work well with lighting changes, blur, and is not the best choice for real time applications as it becomes more computationally expensive as a function of the number of features. In our case, the algorithm was not able to perform as well due to issues with deformity and the object not having many features. 

<p align="center">
  <img src="assets/images/lab4/SIFT_RANSAC.png" img width="460" height="300">
</p>

<p align="center">
  <b> Figure 1: Extracting features and computing putative matches (closest descriptor) </b>
</p>

Template matching: A technique to find areas of an image (source) that match template image (patch). As the goal is to detect the highest matching area, the template image is compared to the source by sliding it across the source one pixel at a time (left to right and up to down). This comparison score for every location is stored in a resulting matrix, and those that are above a defined threshold are marked as detected. The TM_CORR_NORMED was the method chosen for matching. This method works regardless of the size of the template relative to the size of the matched region in the source. However, it often does not perform as well due to differences in orientation, illumination, occlusion, or point of view between the template and source images.

Color Segmentation: A simple object detection algorithm that identifies objects based on the specific object’s color. The pixels were first converted to from BRG to HSV and for the purposes of detecting the orange cone, it’s hue was defined as 20-40 deg. The algorithm goes through every pixel and if a pixel matches a specific ratio of red:blue:green adds it to a set of pixels. It then uses clusters of these pixels to identify contours of objects of that specific color. Finally, using these contours, one can identify the desired object. For our algorithm we used cv2.morphologyEx() to apply morphological transformation to find the pixels in of our desired color and cv2.cv2.findContours() to apply clustering to find the contours of matching objects. To select the contour to make the bounding box around we took the contour with the largest area. While this algorithm is computationally efficient and has little problem dealing with occlusions, it does have issues with illumination, complex objects (e.g. objects with multiple colors), or any time other objects of similar color are in the source image.

**Image Transformations**

With the cone identified, its location in the real world relative to the car was determined using image transformations.  Because the camera was mounted upside down, we flipped the image along the y-z plane, then the x-y plane to achieve the correct visual perspective of the environment.  Using this corrected image, we calculated a homography matrix to transform pixel coordinates to real world coordinates.

The idea behind the homography matrix is as follows: planar homography relates the transformation between two planes, such as the image plane from the camera view and the real world ground plane. This is useful in not only retrieving the camera displacement, but also in mapping camera pixel values to relative positions. OpenCV has a method, `findHomography`, which calculates the homography matrix given a set of source points (camera pixel coordinates), and a set of destination points (corresponding real world coordinates): 

\[
\begin{bmatrix} x^{'} \\ y^{'} \\ 1 \end{bmatrix} = H \begin{bmatrix} x \\ y \\ 1 \end{bmatrix} = \begin{bmatrix} h_{11} & h_{12} & h_{13} \\ h_{21} & h_{22} & h_{23} \\ h_{31} & h_{32} & h_{33} \end{bmatrix} \begin{bmatrix} x \\ y \\ 1 \end{bmatrix}
\]

Specifically for our purposes, our homography matrix was decomposed into an intrinsic parameter matrix and the camera pose, which included the camera’s distortion coefficients: 

\begin{align*} s \begin{bmatrix} u \\ v \\ 1 \end{bmatrix} &= \begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{bmatrix} \begin{bmatrix} r_{11} & r_{12} & r_{13} & t_x \\ r_{21} & r_{22} & r_{23} & t_y \\ r_{31} & r_{32} & r_{33} & t_z \end{bmatrix} \begin{bmatrix} X_o \\ Y_o \\ Z_o \\ 1 \end{bmatrix} \\ &= \boldsymbol{K} \hspace{0.2em} ^{c}\textrm{M}_o \begin{bmatrix} X_o \\ Y_o \\ Z_o \\ 1 \end{bmatrix} \end{align*}

With our homography matrix $$H$$, we converted the cone’s bottom edge midpoint in the camera image to physical coordinates. An example of this is shown in Figure 2. 

<p align="center">
  <img width="460" height="300" src="assets/images/lab4/pixel_to_position.png">
</p>
<p align="center">
<b> Figure 2: Converting Pixel Location to Position.  The bounding box of the cone is returned as pixel coordinates within the picture by the cone detecting algorithm.  Using the calculated homography matrix, the cone’s location is converted to positional data, which is displayed above the bounding box as x,y coordinates in the real world.</b>
</p>

**Cone Parking Algorithm**

With the location of the cone identified in the car’s relative frame, the position of the cone was used as the input to a parking controller. The two coordinates were used to calculate the euclidean distance from the robot to the cone, as well as the required turning angle. The logic of the cone parking algorithm is shown in Figure 3.  When moving forward (cone is in reachable space), the velocity was set with a PD controller, with proportional and derivative gains of 3.0 and 0.1, respectively.  When moving backwards, the velocity was held constant at a low speed (0.3 m/s).  This was chosen because there is no sensory data taken behind the car so it would not be smart to move quickly in that direction.  To get a turning angle, the formula $/phi = arctan(L/R)$ was used, where $/phi$ is the turning angle, $L$ is the length of the car between axles, and $R$ is the turning radius.  This is taken from the dynamics of ackermann steering.  Length is known, so the cone’s y-coordinate was used for the turning radius.  This calculation showed good results in simulation, allowing the car to smoothly approach the cone.

However, this control scheme does produce S-shaped curves during the approach to the cone.  To prevent shaky steering movements in the inflection point of the S-shaped curve, the previous steering angle and current extreme steering angle were averaged if the rate of change of steering angle was too severe (difference between subsequent steering angles > 0.3).

<p align="center">
 <img width="460" height="300" src="assets/images/lab4/car_parker.png">
</p>
<p align="center">
 <b> Figure 3: Illustration of Logic in Car Parker. Here r is the minimum turning radius of the car, d is the desired distance to the cone, and 𝛼 is the desired angle of view that the cone should end up in after parking.  First, the car checks if it is too close to the cone (red circle), backing up if so.  Next, it checks if the cone is reachable with only forward motion (any x,y coordinates above green line), in which case it moves forward with a calculated turn angle.  Otherwise, it reverses until the cone sits in the reachable zone in its relative frame, at which point it can proceed forward to the cone.</b>
</p>

**Line Following Algorithm**

Our car, using a method called setpoint control, was able to perform basic line following using similar principles used to perform cone parking. The color segmentation algorithm worked for cone detection because the line was defined by a continuous strip of tape of the same color. By ‘chopping’ the camera’s view to a specific slice, as illustrated in Figure 4, the car followed the line by continuously pursuing the orange portion of the slice. The same parking controller node used for parking identified the orange tape as the equivalent of a cone at a fixed distance away. As long as the chopped image contained some part of the orange line, the car effectively ‘followed’ the line to the end.
 
<p align="center">
  <img width="460" height="300" src="assets/images/lab4/image_chopper.png">
</p>
<p align="center">
<b> Figure 4: Illustration of ImageChopper methodology. The left image shows a line to be followed. The right image shows the ‘chopped’ image output of the ImageChopper node. Also shown is an example of the bounding box created by ConeDetector.</b>
</p>

We chose to forgo other potential methods for line following, such as trajectory tracking, primarily because of concerns over slow performance due to the complexity of such an algorithm. Setpoint control largely achieves our goal of following a line. One observed issue with this method, however, was a tendency for the car to become unstable when following lines at higher speeds. We sought to minimize this experimentally by manipulating the size of the image slice and adjusting our PD controller gains. Our original chopper used a slice size 0.1 times the height of the ZED camera output image, with the top of the slice located 70 percent of the way down the output image.

## ROS Implementation - [Sean Kelley, Rose Wang]

We chose to implement our technical approach using three ROS nodes: ImageChopper, ConeDetector, and ConeParker.  This strategy was developed in order to avoid a complicated system architecture while allowing for code development to occur in parallel.  A schematic of the chosen system architecture is shown in Figure 5.  
 
<p align="center">
  <img width="550" height="350" src="assets/images/lab4/ros_architecture.png">
</p>
<p align="center">
<b> Figure 5: Schematic of the ROS system architecture.  Nodes are shown in light grey boxes labeled by their associated python file names.  Red arrows indicate message transfers, with text labelling the message type.  Dark grey boxes are python functions called by the associated ROS node.</b>
</p>

ConeDetector is responsible for using the ZED camera’s published Image message to publish the location of the cone in the real world.  This node calls two functions: the color segmentation cone detector function and a function which uses a homography matrix to convert between coordinate frames. Subscribing to the camera, the node publishes two messages: a single coordinate point, as an array, which marks the center of the cone in real world coordinates, and the cone’s bounding box, in diagonal coordinates, in the image coordinate frame for visualization purposes.    

ConeParker subscribes to the real world coordinate topic published by ConeDetector and uses a PD controller to find, and park in front of, the cone at a desired distance by publishing an AckermannDriveStamped message to the vehicle.

To adapt these nodes for the line following application, an ImageChopper node is added to the architecture prior to the ConeDetector.  This node takes a selected slice of the camera image, thereby simulating an orange “cone” that remains at the same distance from the car, and results in the car continuously following the “cone”. This augmented workflow is shown in Figure 6.

<p align="center">
  <img width="550" height="350" src="assets/images/lab4/ros_architecture2.png">
</p>
<p align="center">
<b> Figure 6: Augmented system architecture to adapt the ConeParker for line following.  The ImageChopper selects a horizontal slice of the image that makes the line appear like a cone that remains perpetually at the same distance from the car.  In this way, the ConeParker can be ‘tricked’ into thinking the cone continuously moved away from the car.</b>
</p>

## Experimental Evaluation - [Lucas Novak, Sean Kelley, Rose Wang, Marwa Abdulhai]

### Testing Procedure- [Lucas Novak, Sean Kelley, Rose Wang, Marwa Abdulhai]
Our testing procedure was designed to determine the performance of each of the four tasks previously mentioned.

**Comparing Performance of Detection Algorithms**

In order to determine the most favorable object detection algorithm, we tested the three strategies concurrently on a set of test images.  The accuracy of the resulting bounding boxes in each test image were numerized as a percentile, with 0 output for no identified cone and 1 output for a perfectly matched cone.  For thoroughness, we tested the algorithms on twenty test images with the cone placed in different positions relative to the car and surrounding objects.  The tests were continued repeatedly with small changes to the code in order to achieve the highest possible average percentile for each algorithm.  An example of an output from one of these tests is shown in Figure 7.   

<p align="center">
  <img width="460" height="300" src="assets/images/lab4/cone_detector.png">
</p>
<p align="center">
<b> Figure 7: An example of a proper output of a cone detecting algorithm (in this case template matching) on one of the test cases 
</b>
</p>

The algorithm with the worst performance was SIFT-RANSAC, as it was unable to find a correct bounding box for any test and only scored above a zero on one test due to one corner being near (0,0). This was due to the fact that the cone did not have many features or corners, so SIFT would not detect enough features in the test images to exceed the minimum threshold.   The results from these tests are shown in Table 1.   

The max source image had 5 detectable features, while the template itself had 13. To potentially address this problem, we also tested lowering the threshold, but this also failed because RANSAC requires a substantial number of features. To confirm that the algorithm worked, we tested it on a more complex object by matching the starbucks logo to a couple of their storefronts, as shown in Figures 8 and 9.

<p align="center">
  <img width="460" height="500" src="assets/images/lab4/table1.png">
</p>
<p align="center">
<b> Table 1: Scores of SIFT-RANSAC</b>
</p>

<p align="center">
  <img width="460" height="300" src="assets/images/lab4/SIFT_starbucks.PNG">
</p>
<p align="center">
<b> Figures 8,9: SIFT-RANSAC correctly identifying Starbucks Logo (images taken from simple good image search)</b>
</p>

<p align="center">
  <img width="460" height="500" src="assets/images/lab4/table2.png">
</p>
<p align="center">
<b> Table 2: Scores of Template Matching</b>
</p>

The algorithm that performed the second best was template matching, as it was able to correctly match two of the test cases and come close on three others.  These results are shown in Table 2.  The correct bounding box correspond to tests where the cone exactly matched up to the size and point of view as the template and when there were no occlusions. This can be seen above in Figure 7. In cases where these conditions were not met template matching was unable to make a bounding box, as shown in Figure 10.

<p align="center">
  <img width="460" height="300" src="assets/images/lab4/template_matching.PNG">
</p>
<p align="center">
<b> Figure 10,11: Example of template matching being unable to detect cone due to occlusion (left) and correct identification of bounding box with occlusion with color segmentation.</b>
</p>

The algorithm with the best performance was color segmentation, as it identified the correct bounding box in over 70% of the test cases, as shown in Table 3. The tests did not suffer from occlusion or point of view changes like template matching, as shown above in Figure 11. However, color segmentation did suffer from illumination in some test cases where is saw a different sources of orange, as shown in Figure 12.

<p align="center">
  <img width="460" height="500" src="assets/images/lab4/table3.png">
</p>
<p align="center">
<b> Table 3: Scores from Color Segmentation</b>
</p>

<p align="center">
  <img width="460" height="300" src="assets/images/lab4/color_seg.png">
</p>
<p align="center">
<b> Figure 12: Example of Color Segmentation not identifying cone due to illumination. </b>
</p>

**Testing Parking Controller**

To test the performance of the parking controller, we placed the cone a couple meters away from the car and observed its movement towards the cone to a set distance of 0.4 meters.  While testing our parking controller, we observed that the rate at which our image was being received (20 Hz) was different than the rate the controls being sent to the car (2 Hz). This was due to the fact that we are publishing our control commands after waiting for all of our image processing to cease. We decided to interweave a ROS speed timer to send our values at a specific interval of 20 Hz such that we are processing our image in one thread and determining controller commands in the other. After observing shaky steering movements of our car when trying to turn towards the cone, we incorporated a function that averaged the previous steering angle and current steering angle if the different between them is greater than the max turn angle of 0.3. We also changed the Kp and Kd values to be 3.0 and 0.1 respectively based on experimentation.  

**Testing Line Follower**

While testing our line following algorithm, we noticed that our lower and upper bound color threshold had to be slightly modified for the red line as opposed to the orange hue of the cone. Through a series of initial runs, we noticed a pattern that our car receives images and processes its commands with a large time disparity. This lead to us modifying the height of the image snippet, in order to account for the time delay, and tuning our gain parameters, to deal with oversaturation. Although the parameter adjustment allowed our car to follow the line very accurately, we were unable to see similar performances at higher speeds due to the aforementioned time lag. After timing each step in our workflow, we believe the time lag is due to the two necessary image flips along the y-z plane, then the x-y plane to account for the racecar platform’s incorrect camera placement. 

### Results- [Sean Kelley, Rose Wang, David Klee]

Based on our experimental results for the cone detection algorithm tests, the color segmentation strategy was chosen due to comparatively better results to the alternatives, performing with an average of 70% accuracy.  SIFT RANSAC underperformed due to the fact that a cone does not have many unique features to be distinguishable with a minimum number of features required by RANSAC.  Template matching was more successful than SIFT-RANSAC, but failed in identifying the cone in the majority of test cases due to differences in orientation to the template image. The results of the final comparative test between the three algorithms is shown in tables 1,2,3.

**Cone Parker Algorithm**

Our car parker algorithm ensures that the car parks directly in front of the cone at a desired distance. To do so, it adjusts the car in three different states “stay put”, “reverse turn” , or “go forward” which alternate in order to minimize the angle between the car and the cone. While testing our original algorithm, we realized that our algorithm was too stringent on the car standing at a perfect angle (~5-10 degrees) relative to the cone and we decided to relax the car-to-cone intercept as well as smooth out our steering angle update by averaging. This drastically minimized the car’s rapid transitions between states, and enabled our car to smoothly park in front of the cone. 

**Line Following Algorithm**

The goal of the line following algorithm was to allow the car to trace a strip of orange tape across the ground.  In identifying the position of the orange tape in front of the car, the algorithm performed very well.  After tuning the bounds placed on hue degree, our color_segmentation function could consistently and accurately track the location of the tape.  This was achieved despite the tape having a glossy appearance, which can often confuse color segmentation algorithms due to reflection of nearby surface hues.  Always maintaining knowledge of the tape’s location was important for stable following behavior.  The line following action, which was achieved with PD control, proved successful at lower speeds.  Even after tuning, a proportional gain was not sufficient to follow the line, specifically around sharper curves.  Adding a derivative term improved following behavior around these turns and allowed the car to begin the following without starting directly over and aligned with the tape.  Adjusting the height of the image cropping also influenced wall following behavior.  Isolating only data closer to the front of the car proved more helpful, allowing the controller to react to very local changes in tape location.  Cropping out data higher in the horizon, which could warn the controller of curves coming up further along in time, did not prove to be detrimental to performance, likely because they were no curves in the tape line that turned sharper than the car could (turning radius ~1 m).  

## Lessons Learned - [Rose Wang, Marwa Abdulhai]

### Technical Conclusions - [Rose Wang, Marwa Abdulhai]

This lab exposed us more to the potential workflow pipelines any robust robotic system might face. We realized we need to be more attentive to the runtime of each task and possibly thread independent tasks. Because we need to adjust our camera placement, detect an object of interest and finally send appropriate commands for each new image received in our camera stream, we became more familiar with the bottlenecks in our original system. This incited us to better use the different ROS messages and data types, which we had not yet utilized in the prior lab. Among these include, the ROS Image type, OpenCV Image, the simplified image, and numpy_msgs(Floats). In the future we will be more attentive to efficiently dealing with structuring the car workflow.

We also realized that the lag in the cars movements were as a result of visualizing the car at the same time as running the car on autonomous mode. We were thus able to reduce this effect by not only removing the roslog statements in our code but shutting down the visualization when testing the cars movements once we were certain it could detect the object in front of its view

### CI Conclusions - [Insert Author]

1. [Marwa] From the onset, our team understood that there were many parts of this lab that required timely completion to be passed onto the next step in the workflow. Our team thus took action and divided the various processes in the pipeline. However, we faced issues with making our code as modular as possible and separate from other pieces of code in the pipeline. Since things were very dependent on future portions of the lab, we spent a lot of time debugging. For future labs we are considering having “coding buddies” so people can debug with one other team member early on in the week. But as always, our team is able to communicate well and was committed till the very end and all members were willing to dedicate their time to make things work. 
2. [Austin] After discussing last week’s lab, we made an effort to better assign tasks and to set deadlines for finishing those tasks. This seemed to work better for us at first (it seemed like everyone had an assigned technical task), but one thing that we failed to account for was the dependency of certain tasks on the completion of other ones. The several days we spent debugging our cone detector and cone parking algorithms meant that our attempts to implement a line follower were started late. There could still be improvements to our workflow – things still feel somewhat unorganized in that regard. Still, our team did a great job staying positive and working until the end.
3. [David] This week, we started the week off by creating a spreadsheet outlining all of the necessary tasks that needed to be completed to fulfill the technical and communication objectives of the lab.  This allowed us to delegate tasks more easily and keep a tab on our progress toward our goals.  This was helpful but going forward I think we need to be even more coordinated and organized.  It was not until midway through the week that we all discussed how each piece of code that we had worked on individually would be incorporated together.  This added extra time as we needed to ensure that data flowed correctly in our ROS hierarchy.  I think we are getting better at communicating though and the team synergy has increased as we gain more experience working together.
4. [Rose] We started the lab off well by dividing the lab into independent tasks for each team member to work on. We had multiple discussions of how we wanted to bring together each of our tasks on the high level and tested our parts individually (not on the car), but when we finally brought everything together we spent a lot of time debugging and finding solutions around issues we had with the car itself. I think as a team, we are able to communicate well with each other, take care of logistics as well as organizational items; we were all committed to this project and spent a lot of time together working on it. This week I think we should make an effort in trying to improve our technical collaboration/workflow and ask one of course mentors for advice.
5. [Lucas] Knowing that this lab was going to be a lot of time we started off trying to do things to optimize the workflow. This included dividing up tasks, creating a spreadsheet and checking in with each other more than last week. This did lead us to being more efficient and finishing the lab. For the future we still have a lot to improve of workflow and efficiency. We had a lot of issues combingining all of our individual code together. This slowed us down a lot because a lot of testing, write up and integration of other parts relied on the earlier parts working before we could begin working on them. In the future we need to adjust our workflow such that we aren’t slowing down the pipeline by making eachother’s parts rely on each other in such a linear way. This may mean setting milestones that aren’t just the parts of the lab, being more modular with our process (make code that allows one to check their work and others with the robot early), and still finding better ways to communicate (especially code when one is not present to debug).  
6. [Sean] Based on the performance of our team in this lab, more thorough critical thinking and communication at the beginning is required to work more effectively to the goal.  At the beginning of the week, we thought we had divided up tasks effectively and started quickly since we anticipated the lab to be a lot of work.  However, towards the end we realized that the proposed ROS implementation was overly complicated, which led to problems with data conversion and node communication.  To address these problems, we decided on pairing into three subteams for two reasons: to make task division easier into three sections as opposed to six, and to ensure that any code written is reviewed by at least two members of the team so if one of the writers is unable to attend a meeting, we still understand the logic behind the algorithm.

## Future Work - [Marwa Abdulhai, Rose Wang, Austin Floyd]

Future work for this lab includes refining our color segmentation algorithm to not detect other objects in the image frame that are of similar color, including skin and exit signs. Having this interference proved to be very difficult in detecting our orange cone. 

It would also be interesting to have an image classifier specifically for the cone, which would circumvent the issue we had with objects of a similar shade of cone color. Classifiers such as that the Cascade Classifier from OpenCV increase the relative accuracy of our cone detection algorithms, and in the future, allow for us to do more intricate car controls. This would also differentiate the controls we had for cone parking and line following.

With regards to organization of our code as a whole, it would be better to avoid our current set up of incorporating all functions within one file, which does not allow for much modularity for future labs. Breaking up our code into stand-alone functions and separate files is something we hope to keep in mind moving forward.

Another path for future work includes implementing an improved line follower using trajectory tracking. We chose to forgo this more complex implementation in favor of a simpler, less computationally expensive method using ImageChopper. While our method works well at low speeds, it is less advantageous when moving faster due to increasing instability. A trajectory tracking algorithm might set a lookahead distance, as we effectively did with ImageChopper, fit points on the line to some function, and then find the intersection of the lookahead distance and that function. The car could then travel to that calculated goal point.
