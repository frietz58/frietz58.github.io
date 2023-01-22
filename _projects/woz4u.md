---
title: "WoZ4U: An Open Source Interface for the Pepper robot"
published: true
author_profile: true
teaser: >-
    Study visit at the Intelligent Robotics group in Umeå, Sweden: A comprehensive interface for Softbanks Pepper robot.
header:
  teaser: /assets/woz4u/woz4u.png
toc: true
toc_sticky: true
gallery:
 - url: /assets/woz4u/office.jpg
   image_path: /assets/woz4u/office.jpg
   alt: "Office"
 - url: /assets/woz4u/morning.jpg
   image_path: /assets/woz4u/morning.jpg
   alt: "Nice morning"
 - url: /assets/woz4u/coffee.jpg
   image_path: /assets/woz4u/coffee.jpg
   alt: "Coffee"
 - url: /assets/woz4u/birches.jpg
   image_path: /assets/woz4u/birches.jpg
   alt: "Birches"
 - url: /assets/woz4u/fishing.jpg
   image_path: /assets/woz4u/fishing.jpg
   alt: "Fishing"
 - url: /assets/woz4u/cold.jpg
   image_path: /assets/woz4u/cold.jpg
   alt: "Cold day"


---
<br>
This project presents the results of my three-month study visit at the <a href="https://www.umu.se/en/research/groups/intelligent-robotics/" target="_blank">Intelligent Robotics</a> group at the Umeå University in Sweden: We build a comprehensive and configurable interface for <a href="https://en.wikipedia.org/wiki/Pepper_(robot)" target="_blank">Softbank's Pepper robot</a>.
We initially presented this work at the <a href="https://whisperproject.eu/wasp2020" target="_blank">2020 Workshop on Affective Shared Perception</a> and ultimately published it as <a href="https://www.frontiersin.org/articles/10.3389/frobt.2021.668057/full" target="_blank">artical in the Frontiers in Robotics and AI Journal</a>. As with most of my work, it is <a href="https://github.com/frietz58/WoZ4U" target="_blank">available on GitHub</a>.



## Study visit in Umeå
I initially started working on the Pepper robot in the scope of my student job at the <a href="https://www.inf.uni-hamburg.de/en/inst/ab/wtm/" target="_blank">Knowledge Technology group</a> at Uni Hamburg.
There were already a number of collaborations between the KT group in Hamburg and the <a href="https://www.umu.se/en/research/groups/intelligent-robotics/" target="_blank">Intelligent Robotics group</a> in Umeå, so when I started looking for interesting groups to visit, the Intelligent Robotics group stood out because a) they also work with multiple Pepper robots and b) they were already associated with the KT group. So it came that I visited the Intelligent Robotics group from September to December in 2021 to work on an interface for the Pepper robot, financed through a generous 3-month stipend. The study visit was great, I enjoyed working on this project a lot, everyone at the Intelligent Robotics group was incredibly welcoming, helpful, and friendly, plus there were plenty of oportunities to go camping and fishing on the weekends :)

The most eye-opening experience in this study visit was, by far, going through the lengthy, sometimes tedious process of publishing in a peer-reviewed journal.


<!--
{% include gallery caption="Some impressions form Umeå." %}
-->


## The interface
The idea and requirements behind the interface were formulated by Professor Hellström and Professor Bensch at the Intelligent Robotics group. The main goal was to make Pepper as a research platform for Human-Robot Interaction (HRI) experiments more approachable. This is motivated by the observation that a lot of Pepper's functionality is gated behind the robot's API, which has a steep learning curve and requires sufficient programming knowledge to begin with. Given that there is an inherently social aspect in Human-Robot Interaction, HRI studies carried out by social-science researchers seem valid and required. While these research groups have excellent skills in experiment design, they don't necessarily have the technical expertise to implement the robotic control software required to conduct HRI experiments. Thus, we set out to create an easily approachable, comprehensive, and configurable interface for the Pepper robot, to lower the barrier towards concrete HRI research. We named the interface `WoZ4U`, after the *Wizard-of-Oz* HRI experiment methodology, find it on <a href="https://github.com/frietz58/WoZ4U" target="_blank">GitHub</a>.

<iframe width="560" height="315" src="https://www.youtube.com/embed/Anb5SAnE8Jo" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

### Software architecture
To make the interface as widely accessible as possible, we support all major operating systems. This includes Debian-based Linux, macOS, and Windows. Furthermore, we provide a docker image hosting the interface, which should eliminate most if not all requirement conflicts. The docker image further eliminates the need to follow the lengthy setup guide, as the interface is accessible as soon as the docker container is running on the network. In the backend, the interface is implemented as a webserver. This conveniently makes the interface accessible via web browser from any machine in the network (including smart phones), which removes any requirements towards the OS, since any modern OS comes with a web browser.

We had the non-functional requirement that the interface should be easily (re)configurable for different experiments because a tool specialized for one experiment has no general value for the community. As such, we feature a configuration file in `YAML` syntax. Every non-general part of the UI is configurable through that file so that no programming is required to set up the interface for a new experiment. Instead, one simply edits a few content-specific lines in the configuration file. For example, one might want to investigate which gestures are perceived as particularly friendly. For that, one edits which gestures should be accessible from the interface based on the following snippet:

```yaml
gestures: # Buttons will be created for every item in the list
  -
    title: "Yes"  # This will be shown in the GUI
    gesture: "animations/Stand/Gestures/Yes_1"  # Gesture to execute
    tooltip: "Yes_1 gesture"  # Tooltip for buton
    key_comb: ["shift", "1"]
  -
    title: "No"
    gesture: "animations/Stand/Gestures/No_1"
    tooltip: "No_1 gesture"
    key_comb: ["shift", "2"]
```

This procedure is the same for all elements in the interface so that the entire interface can easily be configured for different experiments or occasions.


### Features
The interface comprises the following robot functionalities:
  + Autonomous life management: Provides controls over the general behavior emitted by the robot
  + Tablet control: Provides controls over which items (pictures, videos, websites) are displayed on Pepper's tablet
  + Speech control: Provides controls over text-to-speech messages
  + Animated speech: Provides controls over speech + gesturing messages
  + LED control: Provides control over Pepper's LEDs
  + Motion control: Provides a simplistic motion controller for Pepper's omnidirectional wheels
  + Gesture control: Provides control over Pepper's gestures

These features comprise almost everything Pepper's API has to offer. In some cases, we even extend the API for some custom functionalities that are not part of the API (audio and touch-event live streams).

Feel free to read <a href="https://www.frontiersin.org/articles/10.3389/frobt.2021.668057/full" target="_blank">our paper</a> for more details.

We hope that this tool is useful for researchers in the HRI field.
