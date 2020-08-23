---
title: "Docker + ROS: How to listen to ROS nodes in Docker containers"
last_modified_at: 2020-08-20 20:50:37 +0200
categories:
  - Linux
tags:
  - Linux
  - Docker
  - ROS
  - VSLAM
mathjax: true
published: false
toc: true
toc_sticky: true
teaser: "Learn how to configure Docker for comminucation with ROS and make incompatible systems work togehter."
---

<h2 id="motivation">Motivation and use case</h2>

Working with Pepper robots at the Knowledge Technology Research Group, we have the high-level goal of building Pepper into a interactive demonstration plattform. One of the brainstormed requirements is that we want a Pepper robot to be capable of autonomously navigate our office space and gather the employees for joint lunch break. To approach the mapping and localization problem at hand, we decided on employing the <a href="https://de.wikipedia.org/wiki/Simultaneous_Localization_and_Mapping" target="_blank"><i> Visual Simultaneous Localization and Mapping </i></a> algorithm. Instead of implementing the algorithm from scratch, we chose the OpenVSLAM implementation by <a href="https://github.com/xdspacelab/openvslam" target="_blank">Sumikura</a> et al, which happens to come with a Dockerfile. Thus, the objective is clear: Get Pepper's sensor data with ROS (possible do some data cleaning), and feed the data to the VSLAM algorithm, which is running in it's Docker container. But getting the two technologies to work hand in hand is only trivial for people who have a deep understand of both frameworks, which I don't yet have. Hence I wan't to share what took me roughly a day to figure out...
<br>

<h2 id="problem">Problem: The missing information in the getting started guides</h2>
Lunch ROSCORE as usual:
<img src="/assets/img/docker-ros/roscore.png">
<br>

Lunch docker image, communic
<img src="/assets/img/docker-ros/fail.png">
<img src="/assets/img/docker-ros/etc_hosts.png">
<img src="/assets/img/docker-ros/fail_after_fix.png">

Consider the following example code:
```bash
ls asd
```

<h2 id="solution">Solution</h2>
ip addr
<img src="/assets/img/docker-ros/ip_addr.png">
roscore --> ip
vim /etc/hosts
export ROS_MASTER_URI
<img src="/assets/img/docker-ros/success.png">




Cheers,<br>
*Finn*.

<br>


<br>
References: <br>
<a href="https://de.wikipedia.org/wiki/Simultaneous_Localization_and_Mapping" target="_blank">[1] Simultaneous Localization and Mapping</a><br>
<a href="https://github.com/xdspacelab/openvslam" target="_blank">[2] OpenVSLAM</a><br>
the asterisk(*) of Python</a><br>
