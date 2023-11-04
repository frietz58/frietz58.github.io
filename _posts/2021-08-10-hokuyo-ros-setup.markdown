---
title: "Hokuyo UST-10LX with ROS Setup"
last_modified_at: 2021-08-10 29:50:33 +0200
categories:
  - Linux
tags:
  - Robotics
  - ROS
  - SLAM
mathjax: true
published: true
toc: true
toc_sticky: true
teaser: "In this post I describe how to set up and configure a Raspberry Pi 3 to collect laser data from the Hokuyo UST-10LX sensor via ROS."
---

<h2 id="motivation">Motivation and introduction</h2>
I have worked for quite a while with Softbank's Pepper robot, which, unfortunately, does not feature the best onboard hardware. Specifically, it only employs three laser sensors that are aimed towards the ground at a slightly awkward angle <a href="#[1]">[1]</a>. These sensors are enough to do minimalistic obstacle avoidance, but not enough for Simultaneous Localization and Mapping (SLAM). As a consequence, at the KT group, we've decided to instead build an external mapping device, using the Hokuyo UST-10LX <a href="[2]">[2]</a> laser sensor. However, the Hokuyo UST-10LX has to be connected to a host machine via ethernet cable to communicate its measurements with a host machine. This requires some network tinkering on that machine, which, in our case, is a Raspberry Pi 3. I found this setup not to be properly documented (or at least I couldn't find anything on this). As such, below I describe how to configure a headless Raspi 3 running Ubuntu Server 20.04 to stream data from the Hokuyo laser sensor into your network. 
<br>

<h2 id="Raspi-setup">Raspi setup and networking</h2>
The reason this sensors usability is not exactly at the <i>plug-and-play</i> level is that it, as already said, communicates with the host machine via ethernet cable, which in turn means that we can't use the LAN port to connect the host machine to our trusted network. This is not necessarily a problem, but if we still want to remotely connect to and work on the said host machine, we have to configure our Raspi in such a way that it a) allows data to come in via the wired network interface while b) connects to the network/internet via the wireless interface. In the following steps, we do just that. Additionally, the ethernet wired network device has to be configured in the right way to communicate with the Hokuyo sensor which, also, I did not find to be well documented.
Note that I did the following steps on the Ubuntu 20.04 server distro, I assume the steps work on all Debian-based Linux systems, but obviously, I did not test that.
<br>

Assuming you've freshly installed Ubuntu Server on your Raspi and plan to use it headless, you'll have to do some initial configuration (including enabling `ssh`), which is widely covered, i.e. <a href="https://pimylifeup.com/ubuntu-server-raspberry-pi/" target="_blank">here</a>. Once that is done and ready, simply log into your Raspi via `ssh`. Assuming your Raspi is currently connected to your network via ethernet cable, we now have to configure it to connect to WLAN, so that the ethernet port is available for the Hokuyo sensor. Do the following steps:
+ Get the name of your wireless network device (likely `wlan0`): 
```bash
$ ls /sys/class/net
# enp8s0  lo  wlan0
```
This outputs the names of all network devices on your system. Your wireless device is likely called `wlan0`, you can read more about network device naming conventions on ubuntu <a href="http://manpages.ubuntu.com/manpages/focal/man7/systemd.net-naming-scheme.7.html" target="_blank">here</a>.
+ Edit the netplan configuration file it is`/etc/netplan/50-cloud-init.yaml with your editor of choice. Add the following lines to it, but make sure they are properly intended (its a YAML file, which is sensitive to this).
```python
wifis:
    <your-wireless-device>:
        dhcp4: true
        optional: true
        access-points:
            "<SSID_WiFi_name>":
                password: "<WiFi_password>"
```
You have to enter your network information, so replace `<your-wireless-device>` with the name of your wireless network device (i.e. `wlan0`), `<SSID_WiFi_name>` with then name of your router and `<WiFi_password>` with the according password. The final file should look similar to this:
```python
# This file is generated from information provided by the datasource. Changes
# to it will not persist across an instance reboot. To disable cloud-init's
# network configuration capabilities, write a file
# /etc/cloud/cloud.cfg.d/99-disable-network-config.cfg with the following:
# network: {config: disabled}
network:
    ethernets:
        eth0:
            dhcp4: true
            optional: true
    version: 2
    wifis:
        wlan0:
            dhcp4: true
            optional: true
            access-points:
                "SSID_WiFi_name":
                    password: "WiFi_password"
```
+ Now, bring up the wireless network card on the Raspi, if it is not already running: 
```bash
$  sudo ip link set <your-wireless-device> up
```
Just executing this won't break anything if your wireless card is already running, but later commands will fail if it is down. Again insert the name of your wireless network device (`wlan0`).
+ Now, run these commands to generate and apply the netplan configuration we entered into `/etc/netplan/50-cloud-init.yaml`:
```bash
$ sudo netplan --debug try
$ sudo netplan --debug generate
$ sudo netplan --debug apply
$ sudo reboot
```
Finally, you can unplug the ethernet cable from your Raspi, and, assuming everything went well, the Raspi will connect to your WLAN after its reboot, in which case you can `ssh` onto it again.

Now, the internet traffic is being handled by the wireless network device and the wired ethernet port is available for the Hokuyo. In the next section, we install the Hokuyo ROS driver to bring up the laser sensor.


<h2 id="ros-setup">ROS and the Hokuyo driver</h2>
The remaining steps are more or less straightforward, but not trivial and not properly documented either (hence this post). Install ROS on your Raspi or the machine you plan to attach the Hokuyo to, by following the <a href="http://wiki.ros.org/ROS/Installation" target="_blank">official installation instructions</a>. Once you have ROS installed, install the ROS-driver for the Hokuyo UST-10LX sensor <a href="[3]">[3]</a>:
```bash
$ sudo apt-get install ros-<ROS-VERSION>-urg-node
```
Here, replace `<ROS-VERSION>` with any of `kinetic`, `melodic` or `noetic`, as these are the only ROS versions currently supported by the package. Alternatively, you can compile the package yourself, but then you will have to take care of setting up the workspace and that all dependencies are satisfied, which I won't describe here, as this process is well documented elsewhere <a href="[4]">[4]</a>. With the drivers installed, now we just start ROS and can get our laser data, right? Well, not really, again because this specific Hokuyo laser sensor communicates with the host machine via ethernet cable. Furthermore, because of that, the instructions given for the `urg_node` package don't work for us either. So, to get laser-data into our <i>handy-dandy</i> ROS topics, we have to assign an IPv4 address to the **wired** network interface (because the sensor is connected via cable), so that the IP address of the Hokuyo sensor falls into the subnet range of the wired network interface. From the <a href="https://www.hokuyo-aut.jp/dl/UST-10LX_Specification.pdf" target="_blank">Hokuyo's datasheet</a> (page 6), we know that the device has the default IP address of `192.168.0.10` assigned. Thus, we can assign an IP address like `192.168.0.15/24` to our wired network device. `192.168.0.15/24` is in CIDR notation <a href="[5]" target="_blank">[5]</a> and states that the first 24 bits (as indicated by the trailing `/24`) provide the network identifier, and whatever comes after that is the actual machine identifier. To assign a specific IP address to the wired network device, take the following steps:
1. Get the name of your ethernet (wired) network device with the same command we used before: 
```bash 
$ ls /sys/class/net
# eth0  lo  wlan0
```
`lo` is the loopback device, `wlan0` (or similar) the WiFi card, so the remaining name is your wired network card (most likely called `eth0`, `eno1` or `enp2s0`). 
2. Assign an IP address to your wired ethernet device that shares the network identifier with the IP of the Hokuyo sensor:
```bash
# sudo ip addr add <SHARED.NETWORK.IDENTIFIER>.<HOST-IDENTIFIER>/24 broadcast <SHARED.NETWORK.IDENTIFIER>.255 dev <ETHERNET-CARD-NAME>
$ sudo ip addr add 192.168.0.15/24 broadcast 192.168.0.255 dev eth0
```
As above, replace the values in `<>`: with your values. You can verify that the command worked by inspecting the output of the `ifconfig` command. For the ethernet device you specified, it should show the IP address we just assigned to it. If your ethernet device does not show up in `ifconfig` but when you run `ls /sys/class/net`, it might be down. You can bring it up the same way we did with the wireless device earlier. 

Now we have our regular traffic running over the wireless connection, while the ethernet wired device is configured to communicate with the Hokuyo sensor. Note, the IP address assignment with the `ip addr add` command we just did is not static, you will have to redo it after rebooting your device. You can look up how to configure this permanently by editing the netplan once again, i.e. <a href="https://linuxize.com/post/how-to-configure-static-ip-address-on-ubuntu-20-04/" target="_blank">here</a>. Now, the final step is to get the laser data into a ROS topic via the ROS driver.

As usual in ROS, first, we start a `roscore` with the simple command `roscore`. Then, you need a new session on the same machine to start the `urg_node` driver for the Hokuyo sensor. Either `ssh` onto the server a second time or use a terminal multiplexer like <a href="https://wiki.ubuntuusers.de/tmux/" target="_blank">tmux</a>. With the roscore running in the other session, run the following command to bring up the Hokuyo laser:
```bash
$ rosrun urg_node urg_node _ip_address:=192.168.0.10
```
With the `_ip_address` argument we pass the IP-address of the physical sensor to the ros node (see <a href="https://github.com/ros-drivers/urg_node/blob/kinetic-devel/launch/urg_lidar.launch" target="_blank">here</a>). As mentioned already `192.168.0.10` is the default value for the Hokuyo UST-10LX, you will have to adjust this if you assigned a different IP address to your sensor. If everything works, the node will output something like:
```bash
[ INFO] [1501672789.034051716]: Streaming data.
```
Now, in yet another terminal session, you can inspect this data, which is the laser point cloud returned by the sensor, by simply calling `rostopic echo /scan`, where `/scan` is the default name of the rostopic used by the `urg_node` driver. 
<br>
<h2 id="rviz">RVIZ</h2>
You can also inspect the laser data in `RVIZ`, but this requires one two steps. RVIZ can only display data that is attached to a valid coordinate frame (which makes sense, because you need <i>some</i> point of reference if you want to visualize things in any space). Usually, the robot you are using will provide the coordinate frames via the ROS `tf` package, but given our current setup, we just have the Hokuyo sensor that is sorta floating in the void. If you inspect the data on the `/scan` topic, for example with `rostopic info /scan`, you will see that the data is associated with the coordinate frame `laser`. But this coordinate frame **does not exist**, so RVIZ can't know where to put this data and will complain if you try to visualize the laser point cloud. To fix this, we simply broadcast a made-up coordinate frame, with the following command:
```bash
$ rosrun tf static_transform_publisher 0 0 0 0 0 0 1 map laser 10
```
Now we would be set, if our `roscore` would run on a system that features a graphical display session, but if you are running the `roscore` on a headless machine, as is the case for the Raspberry 3 Ubuntu server setup describe earlier, we can't even display RVIZ, to begin with. Thus, you will have to do this on a different machine. Luckily, setting up a distributed ROS environment is very easy. Before starting the `roscore` on the headless machine, run the following command:
```bash
$ export ROS_MASTER_URI=http://<IP>:11311
```
Replace `<IP>` with the IP address of the network card. Obviously, when you restart the `roscore`, you will have to restart the `urg_node` as well.
Now, in a terminal on your machine with a graphical desktop environment, run the same command, then `rostopic list`. This should show the rostopics that are running on the headless machine, assuming both machines are in the same network. For some reason, we can now see the rostopics, but no data in the topics, even though we are streaming it from the headless machine. 
I had to apply <a href="https://answers.ros.org/question/90536/ros-remote-master-can-see-topics-but-no-data/?answer=90956#post-id-90956" target="_blank">this weird fix </a> in order to actually see the data in my ros topics on my main machine:
<blockquote>
What finally solved my problem was adding all PCs with their hostnames and IP Addresses to the "/etc/hosts" file. Since then, everything works fine.
</blockquote>
With that being taken care of and all nodes running (`roscore`, `urg_node`, `static_transform_publisher`), you should be able to visualize your laser data in RVIZ. Look at this nice laser data:

<iframe width="560" height="315" src="https://www.youtube.com/embed/hEQP5q-U6MA" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>



<h2 id="summary">Summary</h2>
Alright, in this post we've configured a Raspberry Pi 3 running Ubuntu Server 20.04 to publish laser data into a network via WiFi, while the Hokuyo sensor is connected to the device via ethernet cable. We've installed the required software components and finally showed how to visualize the laser data in RVIZ on a different machine in the same network. Maybe someone finds this helpful :)


Cheers,<br>
*Finn*.

<br>

<br>
References: <br>
<a href="http://doc.aldebaran.com/2-4/family/pepper_technical/laser_pep.html" target="_blank" id="[1]">[1] Pepper laser specification</a><br>
<a href="https://www.robotshop.com/eu/en/hokuyo-ust-10lx-scanning-laser-rangefinder-eu.html" target="_blank" id="[2]">[2] Hokuyo UST-10LX</a><br>
<a href="http://wiki.ros.org/urg_node" target="_blank" id="[3]">[3] URG_node Hokuyo driver</a><br>
<a href="http://wiki.ros.org/ROS/Tutorials/BuildingPackages" target="_blank" id="[4]">[4] Building ROS packages</a><br>
<a href="https://en.wikipedia.org/wiki/Classless_Inter-Domain_Routing" target="_blank">[5] Wikipedia: Classless Inter-Domain Routing</a><br>

