---
title: "How to implement autonomous navigation with Pepper robots"
last_modified_at: 2022-08-31 23:27:11 +0200
categories:
  - Linux
tags:
  - Robotics
  - ROS
  - SLAM
mathjax: true
published: false
toc: true
toc_sticky: true
teaser: "This post describes a solution for autonomous navigation with Pepper robots, which only ship with very limited capabilities in this regard."
gallery:
  - url: /assets/img/pepper-navigation/handheld-0.jpg
    image_path: /assets/img/pepper-navigation/handheld-0.jpg
    alt: Handheld mapping device
  - url: /assets/img/pepper-navigation/handheld-1.jpg
    image_path: /assets/img/pepper-navigation/handheld-1.jpg
    alt: Handheld mapping device (top perspective)
  - url: /assets/img/pepper-navigation/handheld-2.jpg
    image_path: /assets/img/pepper-navigation/handheld-2.jpg
    alt: Handheld mapping device (side perspective)
  - url: /assets/img/pepper-navigation/handheld-3.jpg
    image_path: /assets/img/pepper-navigation/handheld-3.jpg
    alt: Handheld mapping device (bottom perspective)
  - url: /assets/img/pepper-navigation/pepper-frame.jpg
    image_path: /assets/img/pepper-navigation/pepper-frame.jpg
    alt: MakerBeam frame on Pepper robot
  - url: /assets/img/pepper-navigation/pepper-complete.jpg
    image_path: /assets/img/pepper-navigation/pepper-complete.jpg
    alt: Complete MakerBeam and sensor belt on Pepper robot
gallery2:
  - url: /assets/img/pepper-navigation/grove.jpg
    image_path: /assets/img/pepper-navigation/grove.jpg
    alt: Grove hat adapter with connected I2C bus
  - url: /assets/img/pepper-navigation/raspi-grove-imu-hokuyo-anno.jpg
    image_path: /assets/img/pepper-navigation/raspi-grove-imu-hokuyo-anno.jpg
    alt: Raspberry pi with connected grove hat adapter, MPU-9250 IMU and Hokuyo UST-10LX LIDAR
gallery3:
  - url: /assets/img/pepper-navigation/test-env.jpg
    image_path: /assets/img/pepper-navigation/test-env.jpg
    alt: Small maze-like testing environment with unique features for easy mapping and debugging
  - url: /assets/img/pepper-navigation/maze.jpg
    image_path: /assets/img/pepper-navigation/maze.jpg
    alt: Obtained map of testing environment with hand-help mapping device
  - url: /assets/img/pepper-navigation/office_rotated.jpg
    image_path: /assets/img/pepper-navigation/office_rotated.jpg
    alt: Obtained map of entire office
gallery4:
  - url: /assets/img/pepper-navigation/good_map.png
    image_path: /assets/img/pepper-navigation/good_map.png
    alt: A valid map shown in RVIZ.
  - url: /assets/img/pepper-navigation/bad_map.png
    image_path: /assets/img/pepper-navigation/bad_map.png
    alt: An invalid map shown in RVIZ.
---

<h2 id="motivation">Motivation and introduction</h2>
Softbank´s Pepper robot is a popular HRI research platform. As already mentioned in my previous post, their onboard laser sensors hardward supplies very sub-optimal data <a href="#[1]">[1]</a>. Specifically, this data is essentially useless for running SLAM algorithms because a) it is very sparse with only 15 laser beams and b) due to the awkward angle of those beams , the range of those beams is a whopping ~5 meters. I tried `ros-gmapping` with this data and the results were, as expected, not satisfactory. There are two alternatives for achieving autonomous navigation with Pepper robots: One might implement some visual-slam algorithm, i.e. <a href="#[2]">[2]</a>, which we tried, but also found unsatisfactory results due to the shaky, blurry, low-resolution stream of Pepper´s cameras and the relatively featureless environment that is our office building. The last option is building an external mapping device and attaching it to the Pepper, which we did and ultimately found success with. This is far from straightforward and requires a significant time investment, but I hope this post will make the process a little easier for anyone who ever wants to do something similar.

<h2 id="plan">The high-level approach</h2>
The main driver component for our autonomous navigation approach with Pepper robots is the amazing ROS `hector_slam` package <a href="#[3]">[3]</a> from the folks at TUD. Seriously, just look at the amazing results in this video (from 2011, may I add):
<iframe width="560" height="315" src="https://www.youtube.com/embed/F8pdObV_df4" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
Given such results on a handheld device, the overall approach is clear: We just build a similar hardware system as is used in the video, "duct-tape" it onto our Pepper robot and establish communication between the mapping device and Pepper, such that we can map the environment, then run a localization and navigation algorithm to have Pepper autonomously navigate based on the obtained map. Certainly a lot of work but doable. In the following sections, I detail all steps of this approach and attempt to highlight the pitfalls that cost me considerable time along the way.
<br>

<h2 id="hardware">Hardware components</h2>
First of all, if you want to replicate the approach I describe here, you will need to buy a few hardware components:
+ Most importantly, we need a *proper* laser range finder. We experimented with two different sensors: The high-quality Hokuyo UST-10LX (~1300€) and the must cheaper YDLIDAR G2 (~200€). In our office building navigation scenario, we found not significant differences in results between the two, but note that the YDLIDAR G2 only has a range of ~12 meters, which is insufficient for larger spaces.
+ Additionally, the `hector_slam` ROS package not strictly requires but benefits from an inertial measurement unit (IMU), which provides acceleration and angular velocity measurements. These are useful when the mapping device is carried around and not rigidly attached to the pepper robot, in which case there can be significant changes in the alleviation and angle of the device, which must of course be considered by the mapping algorithm. Fortunately, IMUs are relatively cheap, we used the MPU-9250 IMU (~15€).
+ Furthermore, we require a raspberry pi connect get the sensor data from the IMU and the LIDAR. The more ram the better, from 50€.
+ For connecting the MPU-9250 IMU to the raspberry pi we need a special adapter. We used a [grove hat adapter](https://wiki.seeedstudio.com/Grove_Base_Hat_for_Raspberry_Pi/) (~10€) but this component might be optional, depending on the concrete IMU model you opt for.
+ Optionally: You should consider buying a strong battery that power all of the above components, so that you must not attach a power cable to the robot or handheld system when mapping or navigating.
+ Lastly, we need some kind of physical framework to actually attach all of these electronics to. We used a set of [MakerBeams](https://www.makerbeam.com/) (from 100€), alternative a basic set of LEGO bricks might also work ;)
<br>

In total, buying all of these hardware components will cost you about 400€. Considering the original cost of one Pepper robot (14.000€+) this is negligible, especially considering that these few component consideraby enhance the navigation capabilities of the robot.

<h3 id="physical-framework">The physical framework</h3>
Assuming access to all hardware components, I now describe how to set up the raspberry pi with the LIDAR and IMU, so that the data from these two sensors will be available for further processing. We actually ended up building two seperate, physical systems. The first one, which we used mainly for debugging, is a direct adaptation of the hand-held device in the demo video above. The second system is like a "utility belt" for the Pepper robot, that was eventually used to have Pepper autonomously navigate. Here are images of what these systems looked like for us.
{% include gallery caption="Physical mapping & navigation device. Either hand-held or attached to Pepper." %}

<h3 id="lidar">Getting LIDAR data</h3>
Let´s begin by setting up the LIDAR sensor. If you´ve actually opted for the expensive HOKUYU sensor, you can see [an earlier post of mine](/linux/hokuyo-ros-setup/) for how to set it up with ROS. For the second sensor, I will not provide an in-depth guide because its setup and configuration is a relatively straightforward and well document process. However, in short, to get the YDLIDAR G2 to work, I had to
+ Connect the YDLIDAR to the raspberry pi as described in the package manual
+ [install the base SDK](https://github.com/YDLIDAR/YDLidar-SDK/blob/master/doc/howto/how_to_build_and_install.md)
+ [install the YDLIDAR ros driver](https://github.com/YDLIDAR/ydlidar_ros_driver). Don´t forget to replug the YDLIDAR after step 5.

Of course, on the raspberry pi, you should install a version of ROS that is compatible with both the IMU and LIDAR sensor that you bought. Then, at this point, you should be able to run `roslaunch ydlidar_ros_driver G2.launch`, after which the LIDAR should start spinning and data should be visible on the respective ROS topics.
[In this launchfile](https://github.com/YDLIDAR/ydlidar_ros_driver/blob/master/launch/G2.launch), I only changed the name of the laser frame parameter `frame_id` from `laser_frame` to `laser` in line `8`. This is for easier integration with `hector_slam` package later, but not strictly necessary to do this way. This then also requires us to adjust the static transform publisher at the bottom of the file. There, in lines `37` and `38`, the frame name must of course be changed also, and the transformation from the origin to the laster frame must be adjusted so that it reflects translation and rotation from the origin at (0, 0, 0, 0, 0, 0), depending on the design of you physical system. The six values are (x y z yaw pitch roll) from parent frame to child frame. If you are unsure what the static transform publisher does, see section **TODO**.

<h3 id="IMU">Getting IMU data</h3>
With you laser scanner hopefully providing the desired laser data, we now setup the IMU. This of course also depends on the specific IMU device you are using, but the high-level steps of setting up specifically the MPU-9250 IMU are as follows:
+ Connect the IMU to you raspberry pi. If you follow this guide exactly, this involves connecting the grove hat adapter to the `GPIO` bus of the raspberry pi and [installing its driver](https://wiki.seeedstudio.com/Grove_Base_Hat_for_Raspberry_Pi/#installation). Then, connect the MPU-9250 IMU to one of the i2c ports on the grove hat (see picture below).
+ [Install RTIMULib](https://github.com/mryellow/RTIMULib/tree/master/Linux). Pay attention to the step where you allow non-root users access to the i2c bus, I didn´t read this properly at first, which coused cryptic errors to arise further down the line...
+ [Install i2c_imu](https://github.com/jeskesen/i2c_imu). See [this GitHub issue and comment](https://github.com/jeskesen/i2c_imu/issues/10#issuecomment-350224211). [In the MPU launchfile](https://github.com/jeskesen/i2c_imu/blob/master/launch/mpu_9150.launch), for `i2c_imu` with the MPU-9250 we must set param `imu_type` to 7. Additionally, I had to set the value for param `i2c_bus` to 1, which I found out after probing around with the `i2cdetect` bash tool, which can be installed with `sudo apt-get install i2c-tools`.
<figure class="half">
    <a href="/assets/img/pepper-navigation/grove.jpg"><img src="/assets/img/pepper-navigation/grove.jpg"></a>
    <a href="/assets/img/pepper-navigation/raspi-grove-imu-hokuyo-anno.jpg"><img src="/assets/img/pepper-navigation/raspi-grove-imu-hokuyo-anno.jpg"></a>
    <figcaption>Raspberry pi with grove hat adapter and connected MPU-9250 IMU via I2C bus.</figcaption>
</figure>

Now, your IMU should be working, and you should be able to access the sensor data with the raspberry pi, similar to this:
<iframe width="560" height="315" src="https://www.youtube.com/embed/SvgKDOAD_zo" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

<h2 id="hardware-tips">Getting TF right</h2>
When building a physical system like this, ROS TF errors were the temporary bane of my existence. For people with no background in robotics, it might be somewhat unclear what TF actually does, hence I quickly want to provide some background information: When we wish for robots to assume a certain joint configuration, we must look at the current joint values and then calculate the joint velocities that bring us from the current joint configuation to the desired one. This process is called inverse kinematics. These calculations are not terribly hard to do by hand, but of course that´s not what we do, instead we rely on ROS and its various packages for this. For this to work, the robot must provide a valid "kinematic chain", which essentially encode how all the coordinate frame for all the joints are related (i.e. when we move the arm, the attached coordinate frame for the hand will also move).
The following image (from ROS TF2´s website), illustrates this well:

<img src="http://wiki.ros.org/tf2?action=AttachFile&do=get&target=frames2.png">

Why is this relevant? Because ultimately, we must extend the pepper robots coordinate frame, so that it can make use of the external IMU and LIDAR sensors and navigate fully autonomously. Otherwise, the mapping or localization and navigation algorithms can´t make sense of the sensor data they are receiving. Just providing the sensor data without the corresponding coordinate frame link (TF) is not enough, because the algorithm doesn´t know whether the LIDAR sensor is pointing forwad, backward, to the ceiling or straight into the ground. Of course, no all of these configurations are sensible, but you get the idea. The sensor data lives in a certain coordiante frame, and it must be clear how this frame is related to the rest of the robot. This is what the TF package does.

Thus, when mounting the LIDAR or IMU sensor to the robot, make sure you pay attention to local coordinate frame of the sensor, which might or might no be printed onto the sensor. ROS uses a right-handed coordiante systems, so ideally, the sensors coordiante system should align with this, i.e. the z-axis is the vertical one, x-points to the front and the y-axis points to the left. Alternatively, you can correct for wrong coordinate axes when setting up the ROS transforms between the virtual coordinate frames, but  its certainly better practice to set everything up correctly from the get go. Anyway, if you get this wrong, you will quickly notice it, i.e. when the IMU indicates left acceleration when you move it to the right, or because all laser scans are rotated by some degree. In fact, you can see in the IMU video above that the data in rviz is no exactly matching what I do with IMU in the real world. This is exactly the issue, the fixed transform that I specified for testing purposes is not at in-line with the IMU that is just loosely dangling around on my desk, hence the mismatch in orientation.

Thus pay close attention when configuring the static transform publisher with the coordinate frames for our IMU and LIDAR. You can read more about the static transform publisher [here](http://wiki.ros.org/tf#static_transform_publisher) and, if you want to understand this thoroughly, consider chapter 2 and 3 in Bruno Siciliano's "Introduction to robotics" book.  

<h2 id="software-architecture">Software architecture</h2>
Okay, at this point you should have raspberry pi that is running ROS so that the data from the IMU and LIDAR, which are connected to that raspberry pi, is available via rostopics. In principal, this is enough the replicate the mapping results as shown in the hector_slam demo video, via the hector_slam ros package. I describe the mapping process more in-depth [later](#hector_slam), because for now we focus on putting all entities together. That is, we have a main ROS server that will run the computationally expensive algorithms, while the raspberry pi solely provides sensor data and the Pepper robot only executes movement commands. There are different and arguably cleaner ways of implementing this, but the architecture I describe in the following worked quite well for us eventually and gets around the annoying ROS version conflicts, since Pepper´s latest ROS packages support ROS kinetic, yay...


<h3 id="ros-architecture">Distributed ROS </h3>
As mentioned before, we have the following three entities:
+ A central ROS server (with its own `roscore`)
+ The raspberry pi with attached LIDAR and IMU (also with its own `roscore`)
+ A ROS kinetic `roscore` that is running Pepper´s ROS stack

The central ROS server must be able to read the data topics published on the raspberry pi, process that data (i.e. for mapping or localization and navigation), and output movement commands to are forwarded to the pepper robot. In practice, we ran the main `roscore` and Pepper´s `roscore` on a powerful desktop machine, however Pepper´s ROS kinetic core was running in a docker container. This requires that the raspberry pi, the main ROS server machine and the Pepper robot all are in the same network, so that we can setup ROS communication between all these entities. I already [made a blog post](#/linux/ros-docker/) regarding communication with a `roscore` that lives inside a docker container. The key insight is that when we have multiple roscores running in the same network, topics provided by the different roscores can be accessed generally, that is a topic from `roscore` a can be seen by `roscore` b.

<h3 id="pepper-docker">Pepper´s docker ROS kinetic core</h3>
As mentioned, Pepper´s most recent ROS releases are for ROS kinetic (which released 2016). The cleanest solution would be to compile all of those packages manually in a up-to-date ROS environment. What can I say, I tried, it took quite some time, caused me a lot of headache and didn´t even work in the end. Thus, I eventually opted for a docker container running ROS kinetic, where the Pepper´s ROS packages can be installed straightforwardly. In this docker container, simply install [Pepper´s entire ROS stack](http://wiki.ros.org/pepper). Inside this docker container, you should now be able to:
```bash
# launch Pepper´s ros driver
roslaunch pepper_bringup pepper_full_py.launch nao_ip:=<YOUR-PEPPER-IP> roscore_ip:=<DOCKER-KINETIC-ROSCORE-IP>  
# the have it assume wake-up pose
rosservice call /pepper_robot/pose/wakeup
```
of course replacing `<YOUR-PEPPER-IP>` and `<DOCKER-KINETIC-ROSCORE-IP>` with the IP address of the Pepper robot and the local ROS kinetics `roscore` IP address.
With a working ROS kinetic environment that allows us to control Pepper through ROS, we will now make sure that all of our three ROS entities can communicate with each other.

<h3 id="distributed-ros">Key configurations for distributed ROS systems</h3>
There are two key-configurations that are very easy to miss when building a distributed ROS system. I highly consider reading the entire [ROS network setup](http://wiki.ros.org/ROS/NetworkSetup), but nevertheless highlight what these two configurations here.
Firstly, make sure that the IP address to hostname mapping is properly setup in `/etc/hosts`. Specifically, each of the three entities, add the respective other two hostnames and IP addresses to that file. If this is not properly handled, `roscore` a will be able to see the rostopics of remote hostst b and c, but the topics will not contain any data ([see this thread](https://answers.ros.org/question/90536/ros-remote-master-can-see-topics-but-no-data/)).

Secondly, the local time for all three entities must be exactly the same. If I recall correctly, the time in the docker container is identical to that on the host, but certainly the time between the raspberry pi and the desktop machine will not be the same. Here, I mean they must be identical up to a few miliseconds. If they have an offset of e.g. one second, this will very likely cause very weird bugs to occur. For example, I was getting `TF` errors indicating there was some issue with my TF chain, but in fact, the TF messages broadcasted by the raspberry pi where slightly too old (due to the unidentical time on raspberry and desktop), causing the to be dropped by TF running on the desktop. To fix this, install `chrony` on the desktop and on the raspberry pi, the synchronize one machine with the clock of the other (both directions should be fine). This process is also documented [here](http://wiki.ros.org/ROS/NetworkSetup#Timing_issues.2C_TF_complaining_about_extrapolation_into_the_future.3F).

With these things taken care of, you should now be able to access date from the raspberry pi and from the ROS kinetic core on the main ROS server aka desktop. You can test this by visualizing the external LIDAR, the IMU and, for example, Pepper´s laser data in RVIZ on the desktop machine. You should also be able to launch e.g. [rqt-steering](http://wiki.ros.org/rqt_robot_steering) to the desktop and drive Pepper around that way. If this works, your distributed ROS system is working and you should be fine to proceed to the next section.

<h2 id="hector_slam">Mapping with hector_slam</h2>
Given the (hopefully working) distributed ROS system described above, the first step towards autonomous navigation is of course obtaining a map of the environment. As mentioned initially, here we make use of the great `hector_slam` ROS package. Starting the entire system and creating a map of the environment involves the following steps:
+ Start required sensors (execute these commands on the raspberry pi with the attached sensors):
  + Start the IMU (for example using the [mentioned launchfile](#IMU))
  + Start the LIDAR (for example, using the [mentioned launchfile](#lidar))
+ Start Pepper's ROS stack (execute this inside the docker container that communicates with Pepper)
  + Also start your tool of choice to drive the control the Pepper robot, i.e. [`rqt_robot_steering`](http://wiki.ros.org/rqt_robot_steering)
To re-iterate, if you follow this guide strictly, the raspberry pi that collects the sensor data as well as the the docker container that runs Pepper's ROS stack are connected to a central ROS server. To verify that this setup is working correctly, on the central server, you should be able to issue commands to move the Pepper robot while also being able to visualzie (or log) the sensor data collected and published on the raspberry pi.
<br>
Now, we can start the main `hector_slam` ROS package and create a map of the environment.
+ Start `hector_slam` node (this is done on the more powerful, central ROS server)
  + Start [`hector_imu_to_tf`](http://wiki.ros.org/hector_imu_attitude_to_tf), this connects the the LIDAR data to the angles reported by the IMU
  + Start [`hector_geotiff`](http://wiki.ros.org/hector_geotiff), a service for saving the map
  + **Optionally**, start [`static_transform_publisher`](http://wiki.ros.org/tf#static_transform_publisher), to ensure that the TF tree is valid and associates all sensor data with the `base_link`
  + Start [`hector_mapping`](http://wiki.ros.org/hector_mapping), the main `hector_slam` node

Note that there is no difference whether you want to create the map using the hand-held device or with the LIDAR and IMU rigidly mounted to the Pepper robot. The reason for this is the package `hector_imu_to_tf` links the LIDAR scans to the IMU angles. In practise, this is done by configuring the system in such a way that the laser is frame is the `base_stabilized` frame, while this frame is estimated using the IMU data.
TODO, move this elsewhere and reference: http://wiki.ros.org/hector_slam/Tutorials/SettingUpForYourRobot

Thus, if everything works, you should be able to reproduce good results. Here are exemplary mapping results obtained with the above listed hardware. First, we tested and debugged the system in a small, maze-like environment with unique features for easy mapping. Once the system was working well there, we tested it in the main hallway of our office building, and obtained good results.
{% include gallery id="gallery3" caption="A small testing environment and mapping results." %}

Once the entire environment has been mapped, `hector_geotiff` is used to save the map by executing `rostopic pub syscommand std_msgs/String "savegeotiff"`. This did not work for me immediately and complained with *failed with error 'Device not writable'*. I don't know the root cause of this, however, the following command, which re-creates the target folder and takes care of RWX rights fixes the issue:<br>
`sudo mkdir /opt/ros/melodic/share/hector_geotiff/maps &&
sudo chmod -R a+rwx /opt/ros/melodic/share/hector_geotiff/maps &&
sudo chown -R ubuntu:ubuntu /opt/ros/melodic/share/hector_geotiff`.
Of course, you might have to adjust the paths for your ROS installation.


<h2 id="amcl">Navigation and Adaptive Monte Carlo Localization</h2>
Given a map of our environment, we can now run Adaptive Monte Carlo Localization (AMCL) to estimate the current robot position given a stream of lase scans.
You can of course use any other localization algorithm, but the [AMCL ROS package generally](http://wiki.ros.org/amcl) works well.

<h3 id="map_server">Loading hector maps</h3>
To start the localization process in a mapped environment, first we must make the map available on a rostopic. The default topic used for this accurately called `map`.
This can be done by calling the `map-server` package, which takes as argument the path to a map file: `rosrun map_server map_server path/to/map.yaml`. This should make your map available, you can check this by either logging the `map` topic or by visualizing the `map` topic in RVIZ. There is a funny bug/feature when we load maps that were created with the `hector_slam` package, which was a pain to figure out. The maps generated by `hector_slam` draw obstacles in blue instead of black (as be seen on the map images above). This causes them to be classified wrongly by the `map_server` package, because with the default parameters, the blue color falls into an undefined range, meaning the `map_server` does not the blue objects are actually obstacles. This is extra hard to spot, because the resulting map still looks almost perfect when visualized in RVIZ:
{% include gallery id="gallery4" caption="The map on the left is valid and can be used for localization and navigation, while the map on the right is invalid and completely useless for localization and navigation. A subtle but devastating difference." %}

To solve this issue, set the parameter `occupied_thresh` of the `map_server` to a lower value, for example, 0.5. The packages converts the color values in each cell to a value between 0 and 1, and pixel values greater than `occupied_thresh` are considered to be an obstacle on the map. With this conversion, the default value of the parameter is to high which causes the blue obstacle drawn by `hector_slam` to not be recognized as obstacles.

With the map being available on the `map` topic, all that's really left to do now is to start the ACML node. ACML has a lot of parameters, most of them I left at their default values. However, the odometry type should be set to `omni` via the `odom_model_type` parameter, since the Pepper robot has an omnidirectional mobile base. Furthermore, the `laser_min_range` and `laser_max_range` parameters should match your LIDAR model. For the YDLIDAR G2, I set them to 3 and 16, respectively. Lastly, make sure to pass the correct topic TF frames to AMCL. This of course is very specific to your TF chain, but most likely you can leave the default values if you didn't change any TF frame names before.

Store the parameter values in a launchfile and launch AMCL. You can either call the AMCL rosservice `global_localization` to get an uninformed prior that spreads to pose particles uniformly over the free space or you can use the RVIZ `2D pose estimate` button to spread the initial particles normally around the curser location on the map. Either way, after having initialized the localization algorithm you can drive back and forth a bit until the history of laser scans allowed the localization algorithm to converge to the correct position on the map. Once that has happened, the Pepper robot is ready to navigate autonomously to any location on the map.

The navigation is done by the ROS package `move_base`. Given a map and the robots current pose on the map, `move_base` will calculate a motion/trajectory from that pose to a given target pose. Once this trajectory has been calculate, `move_base` will output velocity commands on the `cmd_vel` topic that move the robot along that trajectory to the given target pose. The ROS driver of each specific robot must implement the functionality that interprets and executes the given velocity command with the available robot actuators, like Pepper's omnidirectional wheels.

And that is it. With a setup like this, we can exploit the powerful ROS navigation stack to autonomously navigate with (slightly modified) Pepper robots. Here are two videos, that my colleague Phillip Algeuer mader, because I had leave the Knowledge Technology group to start my Ph.D. right when this project come to its conclusion:




<h2 id="summary">Summary</h2>
To summarize, we wanted that our Pepper robots could autonomously navigate our office premises. We found that the onboard laser ranger finders did not allow for satisfactory mapping and localization performance. Thus, we extended the Pepper robot with an additional, better LIDAR and IMU sensor, which allows us to run the powerful `hector_slam` package for mapping, and `amcl` plus `move_base` for localiation and motion planning/navigation. This required a somewhat sophisticated, distributed hardware, software and network architecture that makes all of the above described components communicate with each other.

Cheers,<br>
*Finn*.

<br>

<br>
References: <br>
<a href="http://doc.aldebaran.com/2-4/family/pepper_technical/laser_pep.html" target="_blank" id="[1]">[1] Pepper laser specification</a><br>
<a href="https://github.com/raulmur/ORB_SLAM2" target="_blank" id="[2]">[2] ORB_SLAM2</a><br>
<a href="http://wiki.ros.org/hector_slam" target="_blank" id="[3]">[3] ROS Hecktor SLAM</a><br>
<a href="http://wiki.ros.org/ROS/Tutorials/BuildingPackages" target="_blank" id="[4]">[4] Building ROS packages</a><br>
<a href="https://en.wikipedia.org/wiki/Classless_Inter-Domain_Routing" target="_blank">[5] Wikipedia: Classless Inter-Domain Routing</a><br>
