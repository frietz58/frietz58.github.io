---
title: "Neural activity animated visualization"
published: true
teaser: >-
  Animated visualization of neural activity in different neural networks, made with Python.
header:
  teaser: /assets/img/mpl_animate_project/dark.gif
sidebar:
  - title: Learnings
    text: >-
      &#8226; Matplotlib animation framework<br>&#8226; Continuous Time Recurrent Neural Networks<br>&#8226;
      <b>LOOK AT IT</b><br>&#8226; I want ont :3
#    image: /assets/img/forest.jpg
#    image_alt: "logo"
#    text: "Designer, Front-End Developer"
#  - title: "Responsibilities"
#    text: "Reuters try PR stupid commenters should isn't a business model"
#gallery:
#  - url: /assets/img/forest.jpg
#    image_path: /assets/img/forest.jpg
#    alt: "placeholder image 1"
#  - url: /assets/img/forest.jpg
#    image_path: /assets/img/forest.jpg
#    alt: "placeholder image 2"
#  - url: /assets/img/forest.jpg
#    image_path: /assets/img/forest.jpg
#    alt: "placeholder image 3"
---
<br>
This project presents a piece of software that I wrote for my student employee
job at the Knowledge Technology Research group.

## The problem of plotting data in many dimensions
Every data-scientist knows knows the struggle of visualizing high dimensional data.
Most 2-dimensional plots are fairly easy to read and can be grasped within one quick glimpse over the plot, at least when the plot is well done.
What is also relatively trivial and vastly popular is the practice of plotting different items in one plot, while the items are discriminated either
by color or some well perceivable properly like shape.
However, plots often get much messier and inherently harder to read when we shift them into three dimensions. Not only do the introduce significant perceptual
overload, there is also the fundamental problem that very, very clear trends can be completely shadowed, depending on the virtual camera angle, through which
the plot is represented. The data that I was given consisted of two factors and their development over time as a third factor. Working with time series data is
always interesting, as is always understood to move _linearly forward_ (ignoring Einstein's Theory of Relativity), which makes time series plots always somewhat intuitive.
Thus, the problem at hand is to find a visualization that allows to express the interplay of three factors, all in one plot,
while maintaining the highest amount of readability possible.

## 3D Time series plots
Without wanting to depict my teachers and colleagues badly (for whom I have nothing but respect), one example for a plot that is relatively cumbersome to read plot is given here:
<img class="align-center" src="/assets/img/mpl_animate_project/both-3d-series.png" />
<figcaption>Hard to read 3D time-series plots. Image adapted from <a href="#avctrnn_citation">[2]</a>.</figcaption>

This plot contains some of the data I had been tasked with to visualize. All the data for the other plots stems from the same paper, so I encourage the interested reader to check out the paper [here](https://www.researchgate.net/publication/327691059_Adaptive_and_Variational_Continuous_Time_Recurrent_Neural_Networks){:target="_blank"}. <br>
Given that the visualization was for a conference, not a follow-up version of the paper, we had the opportunity to experiment with animating the results, which brings the plot back into two dimensions, with time being displayed as actual time frames in the animation. The fact that all plots had a time-component fully enabled this, as animations don't really make sense when the data has no underlying temporal component.

## Matplotlib.animation to the rescue
A quick google search revealed that Python can be used as an animation engine, specifically, with the `Matplotlib.animation` API. There are multiple approaches that can be taken to animate a plot with Matplotlib, the one I personally find most intuitive boils down to creating a function that is called for each time step in the animation and sets the data for each MPL artist for the current time step.

Using the `Matplotlib.animation` API, we can visualize the the ealier plot as follows:
<img class="align-center" src="/assets/img/mpl_animate_project/bright.gif" />
<figcaption>Animated version of the earlier 3D time-series plot.</figcaption>

We could have opted for a _smoother_ animation, but we wanted to maintain the discrete time-steps for this specific graph, hence _artificial_ appearence of the animation. This animation already teasers what we can with Matplotlib's animation framework, but there is more:

<img class="align-center" src="/assets/img/mpl_animate_project/3x4_trace.gif" />
<figcaption>Another example for animations done in Matplotlib.</figcaption>

<br>
<br>
<br>
<br>
<br>

References:<br>
[[1] Knowledge Technology Research Group](https://www.inf.uni-hamburg.de/en/inst/ab/wtm/){:target="_blank"}<br>
[[2] Adaptive and Variational Continuous Time Recurrent Neural Networks](https://www.researchgate.net/publication/327691059_Adaptive_and_Variational_Continuous_Time_Recurrent_Neural_Networks){:id="avctrnn_citation" target="_blank"}<br>
