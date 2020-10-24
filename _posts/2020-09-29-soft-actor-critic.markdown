---
title: "Soft Actor Critic: Deep Reinforcement Learning for robotics?"
last_modified_at: 2020-09-29 22:05:37 +0200
categories:
  - Machine Learning
  - Python
tags:
  - Python
  - Reinforcement Learning
  - Robotic
mathjax: true
published: true
toc: true
toc_sticky: true
teaser: "This post addresses real life feasibility of deep Reinforcement Learning algorithms and how the Soft Actor Critic algorithm relates to the problem"
---

<h2 id="motivation">Motivation and introduction</h2>
The Soft Actor-Critic algorithm by Haarnoja et al. <a href="#[1]">[1]</a> has gotten a lot of coverage and attention in 2018 and 2019. And rightfully so. The paper proposes a very elegant solution to the notorious problem of deep reinforcement learning algorithms being too data-hungry for real-world feasibility and supplies very exciting examples illustrating the capabilities of the algorithm in a real-world setting, as can be seen below. Naturally, I was intrigued. While at the point of writing this post, Reinforcement Learning has not yet been featured on this site, it is, after all, my main academic interest and will be at the heart of my masters'  (and hopefully Ph.D.) thesis. Hence, for one of my courses, I decided to write a paper on the Soft Actor-Critic algorithm. In this blog post, I built on that paper <a href="#[2]">[2]</a> and provide some additional examples and insights.
<iframe width="560" height="315" src="https://www.youtube.com/embed/FmMPHL3TcrE" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
<br>

<h2 id="problem">The problem with Deep Reinforcement Learning for real world robotics</h2>
While this post will not address Reinforcement Learning in general, the gist of it is as follows: By executing pseudo-random actions in an environment (or simulation thereof) and *rewarding* good actions, we can have a, for example robotic, agent learn almost any desired behavior. Here, behavior means that we execute the desired sequence of actions to get from some initial state to some goal state. Inherently, this is a very powerful concept, as this makes it possible for robots to learn how to walk, grasp things, play games, engage in dialogue, and pretty much learn to solve any conditional, sequential problem. That's the theory, at least.
<br>
However, as you might have noticed, we are not yet surrounded by intelligent, autonomous robots in our everyday life, in fact, it's still out of the norm to find a robot autonomously cleaning an office space or shopping mall, which indicates that things aren't quite as easy. Many different fields of robotics are still active research areas, just like Reinforcement Learning is still having a central problem, stopping it from being widely employed in real-world robotic scenarios. To be precise, the main problem of deep Reinforcement Learning algorithms for real-world robotics is that they are insanely data-hungry and take ages to converge (ie manage to generate the desired behavior). <br>
Why is this problematic? Well, robots aren't indestructible and in the early stages of learning, Reinforcement Learning agents behave essentially randomly. You can probably imagine what drastic consequences it can have if we just set all motors in our robot to random power levels... Larger robots will fall over, mobile bots might severely ram into obstacles, and drones would crash immediately. If we expose our robots to this kind of behavior for a prolonged period of time, it is almost certain that the robot will suffer significant damage in the process (similar to how toddlers fall over when they begin learning to walk, except here, consequences aren't inevitable breakdown). And this is only one aspect of the issue. When I wrote *insanely data-hungry*, I absolutely meant that. For example, AlphaStar, DeepMind's deep neural Reinforcement Learning algorithm, has been with trained many agents in parallel, for 14 days straight, on 16 Tensor Processing Units (TPU), corresponding to 200 years of real-life training time, **for each agent** <a href="#[3]">[3]</a>... And this is under the employment of state-of-the-art methods to speed up the learning process. <br>
Ignoring that we can't train a single robot in parallel fashion, after 200 years of hypothetical training, you can be sure that the robot would have broken down simply due to all the wear and tear that it would be exposed to in all that time. <br>
An apparent solution is to train the agent in a simulator (which also allows us to parallelize the training process) and then simply put the *behavior policy*  learned in the simulation on a physical robot, operating in the real world. However, the simulators are not yet good enough and fail to accurately represent the real world, which makes the learned behavior policies useless on the real, physical robot. Further, agents trained in a simulator tend to learn things that are hyper-specific to that simulator and don't generalize to the real world. This is referred to as the Sim-to-Real problem and is an active research area in itself. <br>
So as you can see, there are a lot of challenges for real-world Reinforcement Learning. However, the Soft Actor-Critic algorithm tackles the problem at its root and aims to significantly speed up the learning process, to a point where deep Reinforcement Learning methods become feasible in real-world scenarios. Let's explore the intuition behind the algorithm in the following section.

<h2 id="sac_intuition">The intuition behind Soft Actor Critic</h2>
<p>
To gain an understanding into how the SAC algorithm tackles the data inefficiency problem of deep Reinforcement Learning methods, we have to look at the SAC specific reward function that is being employed by Haarnoja et al. However, to begin with, consider the classical Reinforcement Learning object, that describes the general goal of Reinforcement Learning <a href="#[9]">[9]</a>:
$$G_t = \sum^\infty_{k=0}\gamma^k R_{t+k+1}$$
This is the <i>expected discounted return</i> \(G\) at time step \(t\), with a discount factor \(0 \le \gamma \le 1\), so that the reward signal \(R\) from \(t+k+1\) time steps in the future is weighted to be less important than the reward signal at \(t+k\), encoding an aspect of temporal relevance. The reward signal \(R\) is, arguably, the central part of any Reinforcement Learning problem as this guides how the <i>policy</i> (always denoted by \(\pi\)) our agent will learn, by encoding the <i>goodness</i> of any action taken. Generally, the behavior, which is encoded in the policy of the agent, is adapted in such a way that it maximizes the reward function, thus, a well thought out reward function is the key for success in reinforcement learning. Essentially, no matter what, with Reinforcement Learning, we want to accumulate as much discounted reward, aka return \(G_t\) as possible. This is the main objective all Reinforcement Learning methods are subject to.
Formally, the optimal policy \(\pi^*\) is defined as the policy that has the highest expected reward for every action, at every timestep, in every state <a href="#[1]">[1]</a>:
$$\pi^* = \underset{\pi}{\operatorname{argmax}} \underset{\tau \sim \pi}{\mathbb{E}} \left[ \sum^\infty_{t=0} \gamma ^t [r(s_t, a_t)]\right]$$
Here, (\( \tau \sim \pi\)) means that a trajectory of actions (\(\tau \)) has been sampled (\(\sim \)) from the probability distribution of the policy (\(\pi\)). 
Notice that \(r\) is a function, over all states and actions, providing the reward <i>meassure of goodness</i> for every combination of states and actions (at least in simple examples). 
</p>
<p>
Now, the central element in the SAC algorithm is an advanced, general reward function, that contains a second term in addition to the main reward signal <a href="7">[7]</a>:
$$\pi^* = \underset{\pi}{\operatorname{argmax}} \underset{\tau \sim \pi}{\mathbb{E}} \left[ \sum^\infty_{t=0} \gamma ^ t [r(s_t, a_t) + \alpha \mathcal{H}(\pi(\cdot | s_t)] \right]$$ 
The only difference two the original formula for the optimal policy is the term \(\mathcal{H}\), which is weighted by \(\alpha\). \(\mathcal{H}\) encodes the entropy of the policy \(\pi\) in every state and is given by \(\mathcal{H}(P) =  \underset{x \sim P}{\operatorname{\mathbb{E}}} [-log P (x)]\). Entropy is, roughly speaking, a meassure of information gain or uncertaintaniy of a random variable \(x\), sampled from a distribution \(P\). Do you see what this motivates the Reinforcement learning agent, who behaves according to a learnt policy that maximizes the given function, to do? It forces the agent to not only consider the reward associated with an action in a state, but also the degree of uncertainty in the followup state. 
This results in the agent choosing actions that lead to states which have not yet been seen, especially when a different action would lead to a state that has a higher expected return (but has already been seen). 
The parameter \(\alpha > 0\) balances the two components of the objective function and controls the importance of the entropy term, compared to the reward signal. 
In the original version of the SAC algorithm, this parameter \(\alpha\) had to be set manually, which was a non-trivial problem for complex enough environments and required an expensive hyperparameter optimization <a href="#[1]">[1]</a>. However, in the newer version of the algorithm, Haarnoja et al. managed to automatically adjust the parameter by rephrasing the objective function once again. However, the details of this automatic temperature adjustment can be ignored for the purpose of this blog post.
</p>
<iframe width="560" height="315" src="https://www.youtube.com/embed/KOObeIjzXTY" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
<br>
<p>
In addition to speeding up the overall learning process and making for better data efficiency, this RL objective function has another desirable side effect: It produces much more stable policies <a href="#[1]">[1]</a>, <a href="#[7]">[7]</a>. Unfortunately, it is not further explained why that is, but I think about it like this: Since the reward for every state also depends on the entropy component, the agent is less likely to visit the same state twice because the entropy for that state will already be decreased. Hence, by exploring many, slightly different trajectories (sequences of states and selected actions), the overall policy is more robust, because it does not hinge on observing a small amount of <i>key</i> states in order to be able to select the overall <i>best</i> action. I hope that makes sense...?
But those are just my two cents... Either way, in the video above, we can observe the consequences of this: The agent can deal with significant perturbations of the state (brick wall, stairs, ramp) that it has not encountered during training. This is a very nice property to have, as it implies that the learned policy is more <i>general</i> and can be employed in contexts that are not part of the training data.
</p>
<p>
And this is how far I will go regarding the basic idea behind the SAC algorithm. To summarize, SAC incorporates an entropy term into the Reinforcement Learning objective function, which motivates the agent to select actions under consideration of the uncertainty associated with each state. Like this, the agent can explore the environment much more efficiently, which results in significantly faster convergence, compared to many other state-of-the-art algorithms (see the original paper for benchmarking results <a href="#[1]">[1]</a>). 
For the remainder of this post, we will explore and discuss how the algorithm performs on a practical OpenAI Gym task.
</p>

<h2 id="gym">OpenAI Gym example</h2>
<figure class="half">
    <a href="/assets/img/rl_sac_analysis/bipedal_walker.png"><img src="/assets/img/rl_sac_analysis/bipedal_walker.png"></a>
    <a href="/assets/img/rl_sac_analysis/bipedal_walker_hc.png"><img src="/assets/img/rl_sac_analysis/bipedal_walker_hc.png"></a>
    <figcaption>Figure 1: OpenAI Gym Bipedal Walker environment. Left: Normal version. Right: Hardcore version. </figcaption>
</figure>

OpenAI Gym <a href="#[4]">[4]</a> provides a wide array of Reinforcement learning environments and is one of the de-facto tools being used to benchmark, compare and develop Reinforcement Learning algorithms. For getting practical experience with the SAC algorithm, I selected the BipedalWalker environment, where the goal is for a bipedal agent to develop an efficient walking gait. This environment is particularly interesting, for reasons further explained below, because it has a *normal* and a *hardcore* version, where the hardcore version of the environment contains many stumps, pitfalls, and stairs and is much harder to solve successfully. 
As we can see in the above video, the walking gait learned on the minotaur robot appears to be outstandingly stable, generalizing to a handful of unseen scenarios: The brick wall and the ramp. So my hypothesis is as follows: The bipedal walker trained on the normal version of the environment might be robust enough to also solve the hardcore version of the environment, similar to how the minotaur in the video could deal with the obstacles presented in the testing scenarios! To investigate this hypothesis, we need a working version of the algorithm though. Instead of implementing this algorithm from scratch (which would take a lot of time and straight-up not be efficient), we will use the implementation provided <a href="#[5]">in this repository [5]</a>. 

<h3 id="results">Results</h3>
To begin with, I trained a SAC agent for 500 epochs on both the normal and hardcore version of the environment. For comparison, I also trained a PPO <a href="#[6]">[6]</a> agent and a TD3 <a href="#[8]">[8]</a> on both versions of the environment, to put the convergence time of the SAC agent into perspective. To be fair, PPO is an on policy method, which are known to have much worse data efficiency than off-policy methods. Consider the results presented below:

<div style="text-align: center;">
  <img src="/assets/img/rl_sac_analysis/walker_rewards_4.png" class="align-center">
  <figcaption>Figure 2: Training progress of SAC, TD3 and PPO agents on the normal and hardcore version of the BipedalWalker gym environment.</figcaption><br>
</div>

We can observe that on the normal version of the environment, the algorithm converges within roughly ~ 100 epochs of 5000 interactions with the environment per epoch. However, out of the box, the algorithm does not appear to be able to solve the hardcore version of the environment within 500 epochs. Based on this data alone, I can not really draw further conclusions. It is very well possible that with slight adjustments to the hyperparameters, a SAC algorithm could solve the hardcore version of the environment as well. However, not wanting to invest more time into this blog post, I did not bother to conduct an expensive and timely hyperparameter optimization and applied the algorithm with its out-of-the-box configuration to both versions of the environment. Further, we can observe that the TD3 agent learns just as fast as the SAC agent. Again, we can't really conclude anything beyond that this is how these algorithms perform, given this exact scenario and hyperparameter configuration. The benchmarking results presented by Haarnoja et al. do more justice to the efficiency of the algorithm than this small experiment and I highly encourage taking a look at the paper.

Interestingly enough, TD3 struggles to make meaningful progress in within 500 episodes on the hardcore version of the environment as well. This gives some indication of the difficulty associated with this specific environment. As expected, the PPO agent hasn't come close yet to solving the environement, which it would likely do, given more training data

To begin the analysis of our main hypothesis, whether the policy learned by the SAC agent is robust enough to be transferred from the normal version of the environment to the hardcore one, consider the below Figure:

<div style="text-align: center;">
  <img src="/assets/img/rl_sac_analysis/test_data.png" class="align-center">
  <figcaption>Figure 3: Testing rollouts of the SAC policies learned on both versions of the environment. Left: Average reward obtained. Right: Average episode length.</figcaption><br>
</div>

These statistics tell us a few things on how the learned policies perform, already regarding the main hypothesis we sought out to investigate, whether policies trained on the simple version of the environment would be robust enough to also deal with the obstacles presented in the hardcore version of the environment, without encountering them during training. Sadly, a short glance at Figure 3 immediately falsifies that hypothesis. We can observe that when executing the policy trained on the normal version of the environment on the hardcore version, we get an average reward of roughly -100, with low deviations from that value. This is because the environment *punishes* the agent with a -100 reward when it falls over.  Hence, the policy learned on the normal version of the environment is not robust enough to get past the obstacles in the hardcore version and falls over, getting punished with a -100 reward. Further, we can observe that the agent trained on the normal version of the environment does not appear to have a deviation from the reward and episode length. This indicates that the agent performs equally well (or badly) most of the time, contrary to the agent trained on the hardcore version of the environment. There, we can observe a much higher range of values, indicating external factors (aka the obstacles) having an effect on the performance of the agent.
To further verify our conclusion regarding our main hypothesis, take a look at how the agent performs in practice: 

<div style="text-align: center;">
  <video controls="true" allowfullscreen="true" style="width: 100%;">
    <source src="/assets/img/rl_sac_analysis/SAC_EZ_ON_EZ.mp4" type="video/mp4">
  </video>
  <figcaption>Agent trained on the normal version of the environment, tested on the normal version.</figcaption><br>
</div>
As expected, the agent developed a (kinda awkward looking) walking gait, that successfully solves the normal version of the environment by traveling all the way to the end of the level. This is the most efficient gait it found, as applying torque to the joints costs a small amount of reward. 
Regarding our main hypothesis, consider the following video, where we employ the policy trained on the normal version of the environment on the hardcore version:


<div style="text-align: center;">
  <video controls="true" allowfullscreen="true" style="width: 100%;">
    <source src="/assets/img/rl_sac_analysis/SAC_EZ_ON_HC.mp4" type="video/mp4">
  </video>
  <figcaption>Agent trained on the normal version of the environment, tested on the hardcore version.</figcaption><br>
</div>
Here, we can see how the agent struggles to get past the obstacles. This properly rejects (I think conducting a T-Test on the reward distribution would be overkill and is not necessary for this blog post) our hypothesis: A SAC agent trained on the normal BipedalWalker environment is not robust enough to also solve the BipedalWalkerHardcore environment, as already indicated in Figure 3. In hindsight, I see how this is too big of a leap from the normal to the hardcore version of the environment. There is a clear difference to the real-world examples we saw above: In the examples with the stairs, the ramp, and the small brick wall, the robot, controlled by the learned policy, gets away with just sticking to the learned policy. These obstacles don't require dedicated handling, the robot does not have to learn a specific behavior to get past them. The obstacles faced in the hardcore version of BipedalWalker environment can clearly not be handled in the same way. The agent needs to find a distinct strategy for dealing with the different obstacles present in the hardcover version of the BipedalWalker environment.

So what about the SAC agent that has been trained directly on the hardcore version of the environment? Well, take a look...
<div style="text-align: center;">
  <video controls="true" allowfullscreen="true" style="width: 100%;">
    <source src="/assets/img/rl_sac_analysis/SAC_HC_ON_HC.mp4" type="video/mp4">
  </video>
  <figcaption>Agent trained on the hardcore version of the environment, tested on the hardcore version.</figcaption><br>
</div>

As you can see, that agent performs very poorly. However, I am certain that the agent would learn how to get past the different hurdles given a) more training time/data and or b) a hyperparameter optimization for that version of the environment. We can see already that the walking gait, if we can call it that, differs from what was learned on the normal version of the environment. This becomes even more apparent when we visualize the two agents side by side, one having been trained on the normal version of the environment, the other on the hardcore version:
<div style="text-align: center;">
  <video controls="true" allowfullscreen="true" style="width: 100%;">
    <source src="/assets/img/rl_sac_analysis/SAC_EZ_HC_COMP.mp4" type="video/mp4">
  </video>
  <figcaption>Comparison of walking gaits learned trained on the two environment versions, both tested on the normal version.</figcaption><br>
</div>

<h3 id="extra">Extra</h3>
Purely because it's somewhat interesting too look at, here is a video of the walking gaits developed by the TD3, PPP and SAC algorithm (SAC trained on normal and HC environment):
<div style="text-align: center;">
  <video controls="true" allowfullscreen="true" style="width: 100%;">
    <source src="/assets/img/rl_sac_analysis/ALL_ON_EZ_COMP.mp4" type="video/mp4">
  </video>
  <figcaption>Comparison of learned walking gaits by agent from the different algorithms. SAC (HC) is, again, the gait learned on  the hardcore version of the environment, executed on the normal version. </figcaption><br>
</div>

<h2 id="summary">Summary</h2>
Alright, wrapping it up: Deep Reinforcement Learning methods suffer from strong data inefficiency. The Soft Actor-Critic algorithm by Haarnoja et al. tackles this data inefficiency problem of (deep) Reinforcement Learning algorithms, by modifying the reward object to include an entropy regularization term. Haarnoja et al. provide real-world examples demonstrating strong robustness of the developed policies and strong benchmarking results. We sought ought to investigate whether a SAC policy learned on the normal version of the environment would be robust enough to clear the obstacles in the hardcore version of the environment. Our results clearly indicate that this is not the case, for reasons provided above.

Finally, I want to mention that Haarnoja et al. are of course not the only people investigating data inefficiency in deep reinforcement learning methods. Here are a few approaches, in case you want to do some additional googling: Task Simplification, Imitation Learning, Hindsight imagination, Hierarchical Reinforcement Learning... 

All data and code is available <a href="https://github.com/frietz58/sac_blog_stuff" target="_blank">here</a>.

Cheers,<br>
*Finn*.

<br>

<br>
References: <br>
<a href="https://arxiv.org/abs/1812.05905" target="_blank" id="[1]">[1] Soft Actor Critic Algorithms and Applications, Haarnoja et al. </a><br>
<a href="/assets/img/rl_sac_analysis/Finn_Rietz_Soft_Actor_Critic_Deep_Reinforcement_Learning_for_Robotics_Paper.pdf" target="_blank" id="[2]">[2] My coursework paper on the SAC algorithm</a><br> 
<a href="https://deepmind.com/blog/article/alphastar-mastering-real-time-strategy-game-starcraft-ii" target="_blank" id="[3]">[3] DeepMind's AlphaStar</a><br>
<a href="https://gym.openai.com/" target="_blank" id="[4]">[4] OpenAI Gym</a><br>
<a href="https://github.com/createamind/DRL" target="_blank">[5] Createamind DRL: SAC implementation</a><br>
<a href="https://arxiv.org/abs/1707.06347" target="_blank">[6] Proximal Policy Optimization Algorithms</a><br>
<a href="https://arxiv.org/abs/1801.01290" target="_blank" id="[7]">Soft Actor-Critic: Off-Policy Maximum Entropy Deep Reinforcement Learning with a Stochastic Actor</a><br>
<a href="https://arxiv.org/abs/1802.09477" target="_blank" id="[8]">[8] Addressing Function Approximation Error in Actor-Critic Methods</a><br>
<a href="http://incompleteideas.net/book/the-book-2nd.html" target="_blank" id="[9]">[9] Sutton and Barto: Reinforcement Learning: An Introduction</a><br>

