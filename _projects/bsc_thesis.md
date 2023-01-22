---
title: Convolutional features for visual scale estimation
published: true
toc: true
toc_sticky: true
author_profile: true
teaser: >-
  My bachelor Thesis: Scale estimation for online Visual Object Tracking based
  on convolutional features.
header:
  teaser: /assets/img/hiob_project/hiob_teaser.png
  
gallery:
  - url: /assets/img/hiob_project/success_plots.png
    image_path: /assets/img/hiob_project/success_plots.png
    alt: Benchmarking results success plots
  - url: /assets/img/hiob_project/precision_plots.png
    image_path: /assets/img/hiob_project/precision_plots.png
    alt: Benchmarking results precision plots
  - url: /assets/img/hiob_project/RedTeam.png
    image_path: /assets/img/hiob_project/RedTeam.png
    alt: Example size graph
  - url: /assets/img/hiob_project/Twinnings.png
    image_path: /assets/img/hiob_project/Twinnings.png
    alt: Example size graph
---
<br>
This projects presents mainly the work of my bachelor thesis. However, after
submitting my thesis, I had the opportunity to keep on working on the topic, which
produced the final results presented below.  

## My Bachelor Thesis topic

In the broadest categorization, my Bachelor Thesis belongs to the category of [visual object tracking](https://liu.se/en/article/visuell-objektfoljning){:target="_blank"}. Visual object tracking is a fundamental problem in computer vision, which is a vital requirement for many higher-level goals like human-robot interaction, general robotics or autonomous driving. ![HIOB schematic view](/assets/img/hiob_project/hiob_2.jpg)
<figcaption>Illustration of HIOBs frame processing pipeline. Image taken from <a href="#hiob_paper_citation">[3]</a>.</figcaption>

The _Knowledge Technology_ research group at the University of Hamburg (where, at the time of writing, I am working as a student employee) developed a visual object tracker, called HIOB, that tracks purely based on hierarchical, convolutional features (for more details on HIOB, please consider [Stefan Heinrich et al's work](https://www.sciencedirect.com/science/article/pii/S0925231219301523){:target="_blank"}).

For my Bachelor Thesis, titled "Scale Estimation in Visual Object Tracking" (download the PDF [here](/assets/img/hiob_project/FinnRietzBscThesis.pdf)), I was tasked to extend the HIOB tracker, specifically, to implement some form of Scale Estimation. Scale estimation is another basic problem in computer vision, which addresses to problem of recognizing the same objects at different scale levels. Intuitively, this is critical for successful object tracking. The target object for tracking is arbitrary, thus it is very much possible that the object can move in relation to the camera position, which can cause scale distortions in the feature space. The coping with the distortion of the visual features due to scale transitions of the object is the key concern in the Scale Estimation problems. Consider the following image for an illustration of the scale estimation problem:<br><br>
![Illustration of scale estimation problem](/assets/img/hiob_project/dsst.png)
<figcaption>Illustration of the scale estimation problem. Image taken from <a href="#dsst_paper_citation">[4]</a>.</figcaption>

## My work specifically

After extensive literature research, I identified the most promising approach that was compatible with the HIOB tracker to be the Discriminative Scale Space Tracking paper <a href="#dsst_paper_citation">[4]</a, where a scale filter is trained and updated based on multiple image patches, sampled on multiple scale resolution.

<div>
  <img class="align-left" src="/assets/img/hiob_project/dsst_sample.png" />
  <figcaption>Illustration of the DSST scale sample. Image taken from <a href="#dsst_paper_citation">[4]</a>.</figcaption>
</div>
At each time step, the scale of the objected is estimated, based on a [Histogram of oriented gradients](https://de.wikipedia.org/wiki/Histogram_of_oriented_gradients){:target="_blank"}-representation (HOG) of the scaled image patches. Since this is a very generic approach to the problem, which is independent of the implementational details of the main tracker, it was relatively straight-forward to implement the scale pyramid together with the HOG representation and integrate both into HIOBs main pipeline. This alone allowed for an online updating of the scale of the object during tracking.

However, instead of stopping at this point, my thesis supervisor [Dr. Heinrich](https://www.inf.uni-hamburg.de/en/inst/ab/wtm/people/heinrich.html){:target="_blank"} advised me to implement a second approach, with the idea of estimating the scale based on convolutional features directly, without the need for a dedicated scale representation. Thus, I developed an algorithm that estimates the scale of the target object during tracking. Without going into detail, the key idea is that all convolutional features (that belongs to the object) are detected with a certain confidence and that the totality of all feature confidences can be evaluated, reflecting the scale of the object (for an in-depth explanation, see my [Bachelor Thesis PDF](/assets/img/hiob_project/FinnRietzBscThesis.pdf){:target="_blank"}).

<div>
  <img class="align-left" src="/assets/img/hiob_project/heatmap.png" style="width:400px"/>
  <figcaption>Visualisation of some results from the hyperparameter optimization.</figcaption>
</div>
After implementing both algorithms, we conducted an extensive grid-search hyper-parameter optimization, to determine the ideal configurations for both algorithms. Conducting the optimization was not as straight-forward as I had anticipated it to be. I encountered different problems when running the optimization on different GPU-Servers, ranging from connection-timeouts and certificate-timeouts to weird bugs in my _flawless code_ (irony...), which resulted in me having to repeat the optimizations multiple times. Note, that each optimization had to be done for different modes of both algorithms, while one optimization alone took about 120 hours. However, with tools like TMUX, I was able to resolve most of the server-side issues and by parallelizing the experiments over multiple GPUs, I could reduce the time for the optimization significantly.

Finally, once good hyperparameter configurations were found for both algorithms, we conducted a benchmarking experiment on two different datasets, comparing the performance of both algorithms. These revealed that our novel approach, which estimates the object scale purely based on convolutional features, is significantly better than the DSST algorithm on *one of the two datasets*, while also being orders of magnitude faster, in terms of processed frames by seconds, which were incredibly exciting results! Again, for a more in-depth analysis of these results, I refer to my [actual thesis document](/assets/img/hiob_project/FinnRietzBscThesis.pdf){:target="_blank"} and this [follow-up document](/assets/img/hiob_project/FinnRietzBscThesisAddendum.pdf){:target="_blank"}, containing more recent and polished results. However, while the convolutional features appear to work very well and very robustly for most sequences, we identified that occlusion of the object is particularly challenging to adapt to, which makes sense for an online trained model.

This concludes the very brief write-up of my Bachelor Thesis project. Check out some exemplary results below or take a look at the [Github-Repo](https://github.com/frietz58/se_hiob){:target="_blank"}, if you feel like it :)

## Some results
Without providing an in-depth explanation or analysis, I still want to show some results here, as I find them really exciting. Feel free to check out [the addendum to my thesis](/assets/img/hiob_project/FinnRietzBscThesisAddendum.pdf){:target="_blank"}, where these results originate from and are explained accordingly.
{% include gallery caption="Some results for the really curious reader :)" %}

References:<br>
[[1] Visual Object Tracking](https://liu.se/en/article/visuell-objektfoljning)<br>
[[2] PDF of my Bachelor Thesis](/assets/img/hiob_project/FinnRietzBscThesis.pdf)<br>
[[3] Continuous convolutional object tracking in developmental robot scenarios](https://www.sciencedirect.com/science/article/pii/S0925231219301523){:id="hiob_paper_citation"}<br>
[[4] Discriminative Scale Space Tracking](https://arxiv.org/pdf/1609.06141.pdf){:id="dsst_paper_citation"}<br>
[[5] Histogram of oriented gradients](https://de.wikipedia.org/wiki/Histogram_of_oriented_gradients)<br>
[[6] Dr. Stefan Heinrich](https://www.inf.uni-hamburg.de/en/inst/ab/wtm/people/heinrich.html)<br>
[[7] Follow-up to my Bachelor Thesis](/assets/img/hiob_project/FinnRietzBscThesisAddendum.pdf)<br>
[[8] Github Repo of my Bachelor Thesis](https://github.com/frietz58/se_hiob)<br>
