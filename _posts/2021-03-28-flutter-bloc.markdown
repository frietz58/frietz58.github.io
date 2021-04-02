---
title: "Flutter's BLoC package: An implementation guide for beginners"
last_modified_at: 2021-03-28 16:12:37 +0200
categories:
  - App Development
tags:
  - BLoC Pattern
  - Flutter
mathjax: true
published: false
toc: true
toc_sticky: true
teaser: "This post explains Flutter's BLoC Pattern and guides the integration of BLoCs into an existing Flutter project."
---

<h2 id="motivation">Motivation and introduction</h2>
Google's Flutter Software Development Kit (SDK) <a href="#[1]">[1]</a> is a User-Interface (UI) development toolkit, which compiles iOS and Android native, Web and Desktop Apps, from a single codebase, making it a attractive tool for App develoment acros the board. While Google's documentation is very well done and provide many working examples, as soon as one tries to implement non-basic functionalities, things get relatively tricky, fast. One example for this is how Flutter redraws UI elements: Flutter monitors the state of UI elements (called `widgets`), and refreshes the frontend presentation of the element when it detects that the state of the element changed. However, this refreshing of the frontend is embedded deep into Flutter's inner code, and can't betriggered manually. Further, the state of a widget is only accessible from within that widget, and not from the outside, meaning that a developer can not change the state from `widget a`, when something on `widget b` happens. Of course, this can still be achieved through various dirty ways, but flutter is not meant to be used that way and the solution don't scale to larger projects. 

However, being able to manipulate the state of ui elements without directly interacting with them is very important (think about any scenario where applying some setting changes how other parts of the app, that might currently be invisible, change). Thus, the de facto way to achieve this kind of functionality is through the `BLoC` pattern <a href="#[2]">[2]</a>, which has a vastly populat Flutter implementation. While there <b>a ton</b> of guide and tutorials available that aim to explain how to get started with BLoC and largely share the same ambitions as this post, I found most of the to be unsatisfing at at least one point. As such, in this post I presente the guide that I wish I had found when starting to learn about app development in Flutter...

<br>

<h2 id="what-is-bloc">What is a BLoC</h2>
As we have established, the BLoC pattern in Flutter addresses the problem of making the state of UI elements more managable, specifically so that widgets can be changed from the "outside", i.e. from other places in the codebase. <i>BLoC</i> is short for <b>B</B>uisness <b>L,</b>ogic <b>C</b>omponent and is a general design pattern which is not exclusive for Flutter. The central idea behind BLoC is to maintain a clear separation between the <i>presentation</i> and the <i>buisness logic</i> of components, which has obvious benefits with respect to reusability and extendabilty. For Flutter, the BLoC packages provides convenient state management of UI widgets, while maintaining the aforementioned seperation of presentation and logic. <br>

Concretely, the state concept is leveraged from widgets to logical entities, meaning that each logical <i>component</i> in the app is represented individually, and changes on the on the components state are reflected on the widget/UI level causing the UI elements to be redrawn given a change in the higher-level component state. <br>

An example: Say we have build an application for a car-sharing company. This app contains many low-level UI elements, like buttons, images, links, or pages. It is possible to group these widgets into pages or "views", in which case communication between them is still possible via the state of the parent container element, but not beyond that. With the BLoC package, we still have these low-level widgets, but they can be grouped independently of their location in the app, based on the logical component the belong to. Thus, the different, logical components in our car sharing app could be concepts like cars, profiles, or traffic-feeds. Then, whenever the state of the component changes, the UI element is refreshed wherever that component is used.

<br>



<h2 id="summary">Summary</h2>
Alright, wrapping it up: Deep Reinforcement Learning methods suffer from strong data inefficiency. The Soft Actor-Critic algorithm by Haarnoja et al. tackles this data inefficiency problem of (deep) Reinforcement Learning algorithms, by modifying the reward object to include an entropy regularization term. Haarnoja et al. provide real-world examples demonstrating strong robustness of the developed policies and strong benchmarking results. We sought ought to investigate whether a SAC policy learned on the normal version of the environment would be robust enough to clear the obstacles in the hardcore version of the environment. Our results clearly indicate that this is not the case, for reasons provided above.

Finally, I want to mention that Haarnoja et al. are of course not the only people investigating data inefficiency in deep reinforcement learning methods. Here are a few approaches, in case you want to do some additional googling: Task Simplification, Imitation Learning, Hindsight imagination, Hierarchical Reinforcement Learning... 

All data and code is available <a href="https://github.com/frietz58/sac_blog_stuff" target="_blank">here</a>.

Cheers,<br>
*Finn*.

<br>

<br>
References: <br>
<a href="https://flutter.dev/" target="_blank" id="[1]">[1] Flutter </a><br>
<a href="https://bloclibrary.dev/#/" target="_blank" id="[2]">[2] BLoC pattern</a><br> 
<a href="https://deepmind.com/blog/article/alphastar-mastering-real-time-strategy-game-starcraft-ii" target="_blank" id="[3]">[3] DeepMind's AlphaStar</a><br>
<a href="https://gym.openai.com/" target="_blank" id="[4]">[4] OpenAI Gym</a><br>
<a href="https://github.com/createamind/DRL" target="_blank">[5] Createamind DRL: SAC implementation</a><br>
<a href="https://arxiv.org/abs/1707.06347" target="_blank">[6] Proximal Policy Optimization Algorithms</a><br>
<a href="https://arxiv.org/abs/1801.01290" target="_blank" id="[7]">Soft Actor-Critic: Off-Policy Maximum Entropy Deep Reinforcement Learning with a Stochastic Actor</a><br>
<a href="https://arxiv.org/abs/1802.09477" target="_blank" id="[8]">[8] Addressing Function Approximation Error in Actor-Critic Methods</a><br>
<a href="http://incompleteideas.net/book/the-book-2nd.html" target="_blank" id="[9]">[9] Sutton and Barto: Reinforcement Learning: An Introduction</a><br>

