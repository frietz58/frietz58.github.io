---
title: Multimanual Interaction on a 3D Fitt's Law Task
published: true
author_profile: true

teaser: Using an Oculus Rift alongside a Leap Motion, we investigated interaction techniques for virtual hands.
excerpt: Test excerpt
header:
  teaser: /assets/img/multiple_hands_project/teaser.gif
sidebar:
  - title: Learnings
    text: >-
      &#8226; C-Sharp language<br>&#8226; Unity<br>&#8226; Oculus Rift SDK<br>&#8226; Leap Motion SDK
gallery:
  - url: /assets/img/multiple_hands_project/quadmanual-rev-1.png
    image_path: /assets/img/multiple_hands_project/quadmanual-rev-1.png
    alt: "Control group image"
  - url: /assets/img/multiple_hands_project/quadmanual-rev-2.png
    image_path: /assets/img/multiple_hands_project/quadmanual-rev-2.png
    alt: "Scene with two hand pairs"
  - url: /assets/img/multiple_hands_project/quadmanual-rev-3.png
    image_path: /assets/img/multiple_hands_project/quadmanual-rev-3.png
    alt: "Scene with three hand pairs"
  - url: /assets/img/multiple_hands_project/quadmanual-rev-4.png
    image_path: /assets/img/multiple_hands_project/quadmanual-rev-4.png
    alt: "Scene with four hand pairs"
---
<br>
This is a group project we did in our fifth bachelor semester. We were a team of four students and did this project at the [Human-Computer Interaction group](https://www.inf.uni-hamburg.de/en/inst/ab/hci.html){:target="_blank"} at the University of Hamburg.
In case you just want to get a quick idea what this project is about, consider the following video (for results, scroll down):
<iframe width="560" height="315" src="https://www.youtube.com/embed/HfaF3dSAooc" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

## Research question
We wanted to investigate to which extend subjects would prefer being able to control multiple pairs of virtual hands to solve a 3-dimensional [Fitt's Law](https://en.wikipedia.org/wiki/Fitts%27s_law){:target="_blank"} oriented task. We hypothesized that given a physically exhausting task, subjects would prefer to switch between multiple pairs of hands, that were placed conveniently in the scene, instead of performing many challenging hand and arm movements with only their one, natural pair of hands. However, controlling multiple pairs of hands (even though only one pair was active at all times) might feel unnatural and ads significant mental demand to the user. The additional mental demand originates from keeping track of all the hand pairs that are available for interaction and from selecting the desired hand pair to control, given the interaction goal. Thus, for the conducted experiment, we were concerned with the [NASA-TLX](https://en.wikipedia.org/wiki/NASA-TLX){:target="_blank"} score, the [System Usability Scale](https://de.wikipedia.org/wiki/System_Usability_Scale){:target="_blank"} and the [Borg-questionaire](https://www.hsph.harvard.edu/nutritionsource/borg-scale/){:target="_blank"} for our interaction methods. With these questionnaires, we wanted to determine whether the additional mental demand of controlling multiple hand pairs is preferred over the physical strain of executing many short, but over time exhausting interactions.

## Setup and approach.
The 3-dimensional Fitt's Law task was designed in a unity scene, where the finger and hand movement from the participant's real hands was mapped (in real-time) to the virtual hands.
Using the [Leap Motion](https://www.ultraleap.com/){:target="_blank"}, we could seamlessly track the hands of our experiment's subjects and map these hand and finger movements to the virtual hands in the unity scene. With an [Oculus Rift](https://www.oculus.com/rift/){:target="_blank"} Head-mounted display, the subjects were placed in front of the Fitt's Law task.
<img class="align-center" src="/assets/img/multiple_hands_project/setup_hands.png" style="width:90%" id="rl_setup"/>
<figcaption>The real-world experiment setup.</figcaption>
The interaction goal that the subjects had to fulfill was to touch one of eleven spheres, between the activation of a new sphere, the subjects had move their hands into a resting position for a short duration. By having multiple pairs of hands available, where one pair could be activated with one of four selection techniques, the subjects had to perform fewer physical movements to reach the currently active sphere, compared to executing the complete movement with their real pair of hands. Note, that the _currently active_ pair of hands is the only one that executes the movements of the real hands of the subjects, the inactive hand pairs would remain in an inactive state.
See the following images for the setup of the task:
{% include gallery caption="Gallery of scene setups with multiple hand" %}

## Selection techniques
We implemented four different selection techniques, that allow the user to select any of the available hand pairs to control. These four selection methods were:
+ Gaze: The hand pair closest to the user's gaze vector is active.
+ Gaze confirm: The current hand pair remains active until the user confirms the switching to the hand pair that is currently closest to the gaze vector.
+ Voice: The user says the German keyword associated with the desired hand pair (oben, unten, rechts, links).
+ Button: The user cycles through the hand pairs with a foot button.

We hypothesized that the "Gaze" or "Gaze-confirm" selection techniques would be preferred, as they allow for very fast and direct selection of the desired hand pair. We implemented the "Gaze confirm" method because we felt it is not always given that our hands are executing a task that lies directly in our field of view. Selecting hand pairs via voice commands worked well, however, there was always a slight time delay associated with the word recognition API. Finally, we assumed that the "Button" method would not be well received since it did not allow for direct selection of the desired hand pair. It only provided the possibility to cycle through all available hand pairs, until the desired hand pair would be active.

## Results and outcome
A more formal description of the entire project, including a statistical evaluation of our experiment results is provided in [our paper](/assets/img/multiple_hands_project/multimanual_interactio_paper.pdf){:target="_blank"} (unpublished). The results boil down to the following findings: As expected, the mental demand for controlling multiple pairs of hands is higher, compared to just controlling one pair of hands. Surprisingly, our tests did not reveal significant differences in the perceived physical exhaustion between having only one pair of hands and multiple pairs of hands available to complete the task. Further, subjects required more time to complete the task with multiple pairs of hands (independently of the selection method), compared to the control group where they only had their one, natural pair of hands. Nevertheless, roughly 73% of our subjects ($$n=11$$) stated that they preferred solving the tasks with multiple pairs of hands instead of just one. We are unsure where this tendency originates from, given that we found no significant, but only anecdotal evidence favoring our hypothesis that the tasks would be physically more demanding with only one pair of hands. Finally, from the four selection technique implemented, the majority of subjects preferred the "gaze" technique to control which pair of hands should be active and usable. 
Again, for a more _scientifically sound_ analysis of our experiment results, please consider [the paper](/assets/img/multiple_hands_project/multimanual_interactio_paper.pdf){:target="_blank"}.
<br>

References:<br>
[[1] HCI Research group](https://www.inf.uni-hamburg.de/en/inst/ab/hci.html){:target="_blank"}<br>
[[2] NASA-TLX](https://en.wikipedia.org/wiki/NASA-TLX){:target="_blank"}<br>
[[3] System Usability Scale](https://de.wikipedia.org/wiki/System_Usability_Scale){:target="_blank"}<br>
[[4] Borg-questionaire](https://www.hsph.harvard.edu/nutritionsource/borg-scale/){:target="_blank"}<br>
[[5] Fitt's Law](https://en.wikipedia.org/wiki/Fitts%27s_law){:target="_blank"}<br>
[[6] Leap Motion](https://www.ultraleap.com/){:target="_blank"}<br>
[[7] Oculus Rift](https://www.oculus.com/rift/){:target="_blank"}<br>
[[8] Our unpublished paper for this project](/assets/img/multiple_hands_project/multimanual_interactio_paper.pdf){:target="_blank"}<br>
[[9] Project video](https://www.youtube.com/watch?v=HfaF3dSAooc){:target="_blank"}<br>
