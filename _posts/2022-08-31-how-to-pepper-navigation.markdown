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
---

<h2 id="motivation">Motivation and introduction</h2>
Softbank´s Pepper robot is a popular HRI research platform. As already mentioned in my previous post, their onboard laser sensors hardward supplies very sub-optimal data <a href="#[1]">[1]</a>. Specifically, this data is essentially useless for running SLAM algorithms because a) it is very sparse with only 15 laser beams and b) due to the awkward angle of those beams , the range of those beams is a whopping ~5 meters. I tried `ros-gmapping` with this data and the results were, as expected, not satisfactory. There are two alternatives for achieving autonomous navigation with Pepper robots: One might implement some visual-slam algorithm, i.e. <a href="#[2]">[2]</a>, which we tried, but also found unsatisfactory results due to the shaky, blurry, low-resolution stream of Pepper´s cameras and the relatively featureless environment that is our office building. The last option is building an external mapping device and attaching it to the Pepper, which we did and ultimately found success with. This is far from straightforward and requires a significant time investment, but I hope this post will make the process a little easier for anyone who ever wants to do something similar.

<h2 id="plan">The high-level approach</h2>
The main driver component for our autonomous navigation approach with Pepper robots is the amazing ROS `hecktor_slam` package <a href="#[3]">[3]</a> from the folks at TUD. Seriously, just look at the amazing results in this video (from 2011, may I add):
<iframe width="560" height="315" src="https://www.youtube.com/embed/F8pdObV_df4" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
Given such results on a handheld device, the overall approach is clear: We just build a similar hardware system as is used in the video, "duct-tape" it onto our Pepper robot and establish communication between the mapping device and Pepper, such that we can map the environment, then run a localization and navigation algorithm to have Pepper autonomously navigate based on the obtained map. Certainly a lot of work but doable. In the following sections, I detail all steps of this approach and attempt to highlight the pitfalls that cost me considerable time along the way.
<br>

<h2 id="hardware">Hardware components</h2>
In order to to build our external mapping device, we need to buy a few components.
+ Most importantly, we need a laser *proper* laser range finder. We experimented with two different sensors: The high-quality Hokuyo UST-10LX (~1300€) and the must cheaper YDLIDAR G2 (~200€). In our office building navigation scenario, we found not significant differences in results between the two, but note that the YDLIDAR G2 only has a range of ~12 meters, which is insufficient for larger spaces.
+ Additionally, hector_slam not strictly requires but benefits from inertial measurement unit (IMU) data, that is acceleration and angular velocity measurements. These are useful when the mapping device is carried around, in which case there can be significant changes in the alleviation and angle of the device, which must of course be considered by the mapping algorithm. Fortunately, these are relatively cheap, we used the MPU-9250 IMU (~15€).
+ Furthermore, to actually get the data from these two sensors, we require a raspberry pi (the more ram the better, from 50€).
+ For connecting the MPU-9250 IMU to the raspberry we used a [grove hat adapter](https://wiki.seeedstudio.com/Grove_Base_Hat_for_Raspberry_Pi/) (~10€) but this might be optional depending on the concrete IMU model you opt for.
+ Optionally: You should consider buying a strong battery that power all of the above components, so that you must not attach a power cable to the robot or device when mapping or navigating.
+ Lastly, we need some kind of physical framework to actually attach all of these electronics to. We used a set of [MakerBeams](https://www.makerbeam.com/) (from 100€).
<br>

Thus, in total, buying all of these hardware components will cost you about 400€. Considering the original cost of one Pepper robot (14.000€+) this is negligible and considerably enhances the navigation capabilities of the robot.

<h3 id="physical-framework">The physical framework</h3>
Assuming access to all hardware components, we can now build a physical system that holds all of the electronics. We build two such systems. The first one, which we used mainly for debugging, is a direct adaptation of the hand-held device in the demo video above. Here are images of what these systems looked like for us.
{% include gallery caption="Physical mapping & navigation device. Either hand-held or attached to Pepper." %}

<h3 id="lidar">Getting LIDAR data</h3>
I already described how to setup the Hokuyo laser range finder in [an earlier post](/linux/hokuyo-ros-setup/), thus consider this post if you actually opted for that very sensor. For the second sensor, I will not provide an in-depth guide because its a relatively straightforward and well document process. However, in short, to get the YDLIDAR G2 to work, I had to
+ Connect the YDLIDAR to the raspberry pi as described in the package manual
+ [install the base SDK](https://github.com/YDLIDAR/YDLidar-SDK/blob/master/doc/howto/how_to_build_and_install.md)
+ [install the YDLIDAR ros driver](https://github.com/YDLIDAR/ydlidar_ros_driver). Don´t forget to replug the YDLIDAR after step 5!

Now, on the raspberry pi, you should already be able to run `roslaunch ydlidar_ros_driver G2.launch`. [In this launchfile](https://github.com/YDLIDAR/ydlidar_ros_driver/blob/master/launch/G2.launch), I only changed the name of the laser frame parameter `frame_id` from `laser_frame` to `laser` in line 8 (for this to work with `hector_slam` later) and the static transform publisher at the bottom. There, in lines 37 and 38, the frame name must of course be changed also, and the transformation from the origin to the laster frame must be adjusted so that it reflects translation and rotation from the origin at (0, 0, 0, 0, 0, 0), depending on the design of you physical system. The six values are (x y z yaw pitch roll) from parent frame to child frame.

<h3 id="IMU">Getting IMU data</h3>
With you laser scanner hopefully providing the desired laser data, we now setup the IMU. This of course also depends on the specific IMU device you are using, but the high-level process of setting up the MPU-9250 IMU goes as follows:
+ Connect the IMU to you raspberry pi. If you follow this guide exactly, this involves connecting the grove hat adapter to the `GPIO` bus of the raspberry pi and [installing its driver](https://wiki.seeedstudio.com/Grove_Base_Hat_for_Raspberry_Pi/#installation). Then, connect the MPU-9250 IMU to one of the i2c ports on the grove hat (see picture below).
+ [Install RTIMULib](https://github.com/mryellow/RTIMULib/tree/master/Linux). Pay attention to the step where you allow non-root users access to the i2c bus, I didn´t read this properly at first and skipped it, which of course does not work.
+ [Install i2c_imu](https://github.com/jeskesen/i2c_imu). See [this GitHub issue](https://github.com/jeskesen/i2c_imu/issues/10). [In the launchfile](https://github.com/jeskesen/i2c_imu/blob/master/launch/mpu_9150.launch), for `i2c_imu` with MPU-9250 we must set param `imu_type` to 7. Additionally, I had to set the value for param `i2c_bus` to 1, which I found out after probing with `i2cdetect`, which can be installed with `sudo apt-get install i2c-tools`.
<figure class="half">
    <a href="/assets/img/pepper-navigation/grove.jpg"><img src="/assets/img/pepper-navigation/grove.jpg"></a>
    <a href="/assets/img/pepper-navigation/raspi-grove-imu-hokuyo-anno.jpg"><img src="/assets/img/pepper-navigation/raspi-grove-imu-hokuyo-anno.jpg"></a>
    <figcaption>Raspberry pi with grove hat adapter and connected MPU-9250 IMU via I2C bus.</figcaption>
</figure>

Now, your IMU should be working, with gives us a result similar to this:
<iframe width="560" height="315" src="https://www.youtube.com/embed/SvgKDOAD_zo" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

<h3 id="hardware-tips">Pay attention to device coordinate systems</h3>
I quickly want to highlight one thing that must be considered when assembling the physical system. The LIDAR and the IMU must be mounted in such a way that their real-world coordinate system aligns with that assumed by the ros driver packages. The real coordinate system might or might not be printed on the devices. If it is, pay attention you assemble it such that the z-axis is the vertical one, x-points to the front and the y-axis points to the left (right-handed coordinated system). Alternatively, you can correct for wrong coordinate axes when setting up the ROS transforms between the virtual coordinate frames, but  its certainly better practice to set everything up correctly from the get go. Anyway, if you get this wrong, you will quickly notice is i.e. when the IMU indicates left acceleration when you move it to the right, or because all laser scans are rotated by some degree. In fact, you can see in the IMU video above that the data in rviz is no exactly matching what I do with IMU in the real world. This is exactly the issue, the fixed transform that I specified for testing purposes is not at in-line with the IMU that is just loosely dangling around on my desk, hence the mismatch in orientation.

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
Now we can map with hector_slam.
required ros nodes
saving map

<h2 id="amcl">Navigation with Adaptive Monte Carlo Localization</h2>
Finally, we can load the map.
nasty pitfal









<h2 id="summary">Summary</h2>

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
