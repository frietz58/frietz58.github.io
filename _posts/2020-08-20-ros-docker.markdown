---
title: "Docker + ROS: How to listen to ROS nodes in external Docker containers"
last_modified_at: 2020-08-20 20:50:37 +0200
categories:
  - Linux
tags:
  - Linux
  - Docker
  - ROS
  - VSLAM
mathjax: true
published: true
toc: true
toc_sticky: true
teaser: "Learn how to configure Docker for comminucation with ROS."
---

<h2 id="motivation">Motivation and use case</h2>

Working with Pepper robots at the Knowledge Technology Research Group, we have the high-level goal of building Pepper into an interactive demonstration platform. One of the brainstormed requirements is that we want a Pepper robot to be capable of autonomously navigating our office space and gather the employees for a joint lunch break. To approach the mapping and localization problem at hand, we decided on employing the <a href="[1]"><i> Visual Simultaneous Localization and Mapping [1]</i></a> algorithm. Instead of implementing the algorithm from scratch, we chose the OpenVSLAM implementation by <a href="[2]">Sumikura [2]</a> et al, which happens to come with a Dockerfile. Thus, the objective is clear: Get Pepper's sensor data with ROS (possibly do some data cleaning), then feed the data to the VSLAM algorithm, which is running in it's Docker container. But getting the two technologies to work hand in hand is only trivial for people who have a deep understanding of both frameworks, which I didn't initially have. Hence I want to share what took me roughly a day to figure out...
<br>

<h2 id="problem">The problem</h2>

Looking for how to approach the issue of reading ROS sensor data in Docker containers, I consulted the official documentation. <a href="[3]">ROS's documentation regarding Docker [3]</a>, only shows us how to listen to ROS nodes/topics when the main `roscore` command is run inside the Docker container as well. That is not what I wanted though, because all our other projects were already implemented outside of Docker, we only needed Docker for one component: The VSLAM implementation. The documentation regarding <a href="[4]">Docker's main ROS image [4]</a> didn't help me either. 
Hence, to begin tackling this problem, the first step is clear: `roscore` must be running somewhere, since this is a requirement for every ROS based system:

<div>
  <img src="/assets/img/docker-ros/roscore.png">
  <figcaption>Classical roscore command.</figcaption><br>
</div>

The next, similarly basic, step is to lunch our ROS docker image (in a new terminal), and try to start communicating with the running `roscore`. Following the instruction's from <a href="#[3]"> [3]</a> again, we run the image and source the entrypoint. To test whether the communication between the external `roscore` and our docker image works, we use the `rosnode list` command, which lists all active nodes. Given that `roscore` is running, there should always, at least, be the `/rosout` node. However, as we can see, executing these steps yields `"ERROR: Unable to communicate with master!"`
<div>
  <img src="/assets/img/docker-ros/fail.png">
  <figcaption>No communication with external roscore from docker container.</figcaption>
</div>

Googling for this specific ROS error message reveals interesting and <a href="[5]">helpful threads [5]</a>, that point us into the right direction. Namely, the key problem is that within our docker container, we don't find the `roscore` that is running on the main system. Hence, we need to set the right environment variable (`ROS_MASTER_URI` <a href="[6]">[6]</a>), that indicates where to find the running `roscore`. Luckily enough, the `roscore` command provides us with that information (consider the output from the `roscore` command above). In my case, the ROS master is located at `http://finn-ubuntu:11311/`. A quick look into `/etc/hosts` reveals the IP we that hides behind the local "finn-ubuntu" hostname:

<div>
  <img src="/assets/img/docker-ros/etc_hosts.png">
  <figcaption>/etc/hosts contains mapping from hostnames to IPs.</figcaption>
</div>

However, as we can observe below, even setting the right environment variable within the docker container does not appear to solve the issue, we are still left with the same error as before: 
<div>
  <img src="/assets/img/docker-ros/fail_after_fix.png">
  <figcaption>Still no comunication after setting the right environment variable.</figcaption>
</div>

So what's causing this? Are we approaching the error from the wrong side? Did we maybe just have a typo somewhere? And most importantly: How do we fix this and finally access our valuable ROS nodes/topics from within our docker container? 

<h2 id="solution">The solution</h2>

Actually, the final (working) solution is very close to what we did previously. However, one crucial detail is missing: The fact that docker containers, per default, live in a <a href="[7]">virtual bridge network [7]</a>! The reason for this is explained in <a href="#[7]">[7]</a>: 

<blockquote>In terms of Docker, a bridge network uses a software bridge which allows containers connected to the same bridge network to communicate, while providing isolation from containers which are not connected to that bridge network. </blockquote>
<div>
  <img src="/assets/img/docker-ros/ip_addr.png">
  <figcaption>ifconfig command revealing information about our network.</figcaption>
</div>

While this is certainly a very powerfull and usefull concept, it is also apparent how this caused our earlier fix to fail. Inspecting the output from `ip addr` further illustrates this "problem": We see all the network interfaces that are currently running on our computer. This usually includes at the very least `lo` loopback device (which function as a local virtual network and runs on 127.0.0.1) and the physical network adapter, usually called `en0` or in my case `enp8s0`. <br>
(Read more about <a href="#[8]">network interfaces [8]</a> and their <a href="#[9]">naming convention [9]</a>)<br>
Additionally, we see the `docker0` interface. This is the virtual bridge network mention above. Because of this, docker container can communicate with one another, but are isolated from the other host networks, including `lo`, our loopback device responsible for the local network on our host machine. As we have looked up earler, `roscore` runs on 127.0.1.1 (i.e. is running locally) and thus not visible from the `docker0` bridge. 

However, our docker container of course has access to the internet, meaning we can access `enp8s0` from within docker. Thus, we can access our host machine via its ipv4 address, which is visible form within our subnet (subnet as in the network that all the devices use that are connected to your router).
Knowing that our `roscore` runs on port 11311, we again set the environment variable form within our docker container: `export ROS_MASTER_URI=http://192.168.10.27:11311/`. However this time, instead of using the `lo` network address (127.0.0.1), we used the `enp8s0` ip address of our machine.
And voilà, `rosnode list` displays the the `/rosout` node, which proves that communication with the `roscore`, from within the docker container, works!

<div>
  <img src="/assets/img/docker-ros/success.png">
  <figcaption>Working communication with roscore form within docker container.</figcaption>
</div>

<h2 id="elegant_solution">The more elegant solution</h2>
The above doctrine deducts the solution mirroring my learning experience. However, after identifying the root of the problem and learning about the whole <a href="#[10]">docker networking thing</a>, I now know that there is a much more elegant solution to the problem. Turns out, the docker developers have considered that some people might need to be able to communicate with service running locally on the machine that also hosts/runs docker. For such a scenario, there exists a dedicated network driver, that we can pass to the docker container. This is as simple as passing the <a href="#[11]">following argument</a> to our docker call:<br> `--network host`. And that's is, by adding this argument to the command, you should be able to listen to ros nodes from within you docker containers right away.

<h2 id="summary">Summary</h2>
Per default, docker containers run in a virtual bridge network, isolating them from host networks like `lo`, making the localhost unaccessable. Docker provides a network driver that removes the isolation and makes the loopback device network accessable from within the docker container. This host networking driver can be activated with the `--network host` command line argument. Alternatively, the ioslation can be circumvented manually by using the `en0` network adapter ip address of the host machine (which is accessable within docker container for TCP/IP communication).


Cheers,<br>
*Finn*.

<br>

<br>
References: <br>
<a href="https://de.wikipedia.org/wiki/Simultaneous_Localization_and_Mapping" target="_blank" id="[1]">[1] Simultaneous Localization and Mapping</a><br>
<a href="https://github.com/xdspacelab/openvslam" target="_blank" id="[2]">[2] OpenVSLAM</a><br>
<a href="http://wiki.ros.org/docker/Tutorials/Docker" target="_blank" id="[3]">[3] ROS Docker documentation</a><br>
<a href="https://hub.docker.com/_/ros" target="_blank" id="[4]">[4] Docker ROS image</a><br>
<a href="https://answers.ros.org/question/43981/error-unable-to-communicate-with-master/" target="_blank">[5] Unable to communicate with master fix</a><br>
<a href="http://wiki.ros.org/ROS/EnvironmentVariables#ROS_MASTER_URI" target="_blank">[6] ROS MASTER URI</a><br>
<a href="https://docs.docker.com/network/bridge/" target="_blanck" id="[7]">[7] Virtual bridge network</a><br>
<a href="http://www.aboutlinux.info/2006/11/ifconfig-dissected-and-demystified.html" target="_blanck" id="[8]">[8] ifconfig command in depth</a><br>
<a href="https://unix.stackexchange.com/questions/134483/why-is-my-ethernet-interface-called-enp0s10-instead-of-eth0"  target="_blanck" id="[9]">[9] enpXsxX naming convention</a><br>
<a href="https://docs.docker.com/network/"  target="_blanck" id="[10]">[10] Docker networking guide</a><br>
<a href="https://docs.docker.com/network/network-tutorial-host/"  target="_blanck" id="[11]">[11] Docker host network driver</a><br>

