---
title: "Multi-objective deep reinforcement fearning with lexicographic task-priority constraints"
last_modified_at: 2023-09-10 14:46:12 +0200
categories:
  - Machine Learning
tags:
  - Python
mathjax: true
published: false
toc: true
toc_sticky: true
teaser: "In this post I summarize our recent paper and advocate for less reward engineering in favor of constrained RL."
---

<h2 id="motivation">Motivation and introduction</h2>
Deep Reinforcement Learning (RL) Doesn't Work Yet. 
That's what Alexander Irpan wrote in his [famous blog post](https://www.alexirpan.com/2018/02/14/rl-hard.html) back in 2018. 
Sadly, 5 years later, we, as RL researchers and practinioners, are still pretty much struggling with the same challenges we stuggled with 5 years ago. 
Although we had some exciting advances and innovations, e.g. the [Dreamer algorithm series](https://danijar.com/project/dreamerv3/), [Decision Transformer](https://proceedings.neurips.cc/paper_files/paper/2021/file/7f489f642a0ddb10272b5c31057f0663-Paper.pdf), and [SayCan](https://arxiv.org/pdf/2204.01691.pdf) come to mind, I belive most practitioners would agree with the following statements:
+ DRL algorithms, that learn to solve each task from scratch, are still very sample inefficient, often requiring millions of transitions before achieving acceptable levels of performance.
+ Designing scalar-valued reward functions for complex tasks that induce the desired behavior is very difficulty with very few general heuristics available.
+ The inherent unsafe exploration of trial-and-error based learning algorithms and the blackbox-natures of the resulting DNN-based agents hinders the more wide-spread employment of DRL in real-world applications.

Thus, I am happy to proclaim that with [our recent paper ("Prioritized Soft Q-Decomposition for Lexicographic Reinforcement Learning")](https://arxiv.org/pdf/2310.02360.pdf), we address all of these pain-points, at least to some extent
So with this hopefully haven  otten you interested in our work, in the next sections I will provide an informal summary of our method, starting with on Multi-objective RL (MORL) and Q-Decomposition.
<br>


<h2 id="morl_decomposition">MORL and Q-decomposition</h2>
In our paper, we want to solve special MORL problems, but we begin with a general definition for MORL problems. 
MORL problems are formalized by a markov decision process (MDP), $$\mathcal{M} \equiv (\mathcal{S, A}, \mathbf{r}, p, \gamma)$$, where $$\mathcal{S, A}$$ respectively denote the state- and action-space, $$\mathbf{r}: \mathcal{S} \times \mathcal{A} \to \mathbb{R}^n$$ is a vector-valued reward function, $$p$$ denotes the transition dynamics and the discount factor $$\gamma \in [0, 1]$$. 
Importanty, in MORL, each dimension $$i$$ of the vector-valued reward function $$\mathbf{r}(\mathbf{s},\mathbf{a})$$ corresponds to a scalar-valued subtask reward function, meaning we have $$\mathbf{r}(\mathbf{s},\mathbf{a})_{[i]} = r_i(\mathbf{s},\mathbf{a}) \in \mathbb{R}$$.
Clearly, directly maximizing the vector-valued reward function is not possible, since we can't determine a unique optimal policy.
For this reason, MORL algorithms usually rely on scalarization: 
The vector valued reward function is scalarized via some weighted sum, then the resulting scalar-valued reward function $$\bar{r}(\mathbf{s}, \mathbf{a}) = \sum_{i=1}^n \beta_i r_i(\mathbf{s}, \mathbf{a})$$can be maximized with your RL algorithm of choice. 

A useful technique for solving MORL problems is [Russell and Zimdars Q-Decomposition](https://cdn.aaai.org/ICML/2003/ICML03-086.pdf) formulation. 
The paper states that for scalarizable, vector-valued reward functions, the Q-function can be decomposed into $$n$$ local Q-functions, each corresponding to one subtask. 
This means that for the $$n$$ subtask reward functions in $$\bar{r}(\mathbf{s}, \mathbf{a})$$ and a policy $$\pi$$, we can learn $$n$$ Q-functions 

$$
Q_i^\pi (\mathbf{s}, \mathbf{a}) = r_i(\mathbf{s}, \mathbf{a}) + \gamma Q_i(\mathbf{s}', \pi(\mathbf{s}')), \forall i \in \{1,\dots,n\} \tag{1}
$$

and reconstruct the Q-function for the overall, scalarized MORL problem as 

$$
\bar{Q}^\pi = \sum_{i=1}^n \beta_i Q_i^\pi(\mathbf{s}, \mathbf{a}). \tag{2}
$$

This result is useful, because it allows us to learn these Q-functions seperately and concurrently. 
Furthermore, we can potentially transfer and re-use the constituent Q-functions $$Q_i^\pi(\mathbf{s},  \mathbf{a})$$ for different tasks.
In addition to these points, the decomposed nature of the Q-Decomposition methods benefits interpretability of the RL agent, since we can inspect the different components that jointly induce the behavior of the agent, which is not possible with classical, non-decomposed blackbox agents. 
Due to these positive attributes of Q-Decomposition, we are building on and extending the Q-Decomposition framework in our paper. 
In particular, we want to solve continuous action-space MORL problems (e.g. robot control problems) and therefore exploit Q-Decomposition in MaxEnt RL, which I briefly summarize in the next section.

<h2 id="maxent_rl">MaxEnt RL</h2>
MaxEnt RL, essentially, regularizes the policy by punishing unneccesarily deterministic policies. This is achieved by adding Shannon's entropy $$\mathcal{H}(\mathbf{a}_t \mid \mathbf{s}_t) = \mathbb{E}_{\mathbf{a}_t \sim \pi(\mathbf{a}_t \mid \mathbf{s}_t)}[-\log \pi(\mathbf{a}_t \mid \mathbf{s}_t)]$$ to the reward signal, meaning the optimal MaxEnt policy is given by 

$$
\pi^*_\text{MaxEnt} = \underset{\pi}{\arg \max} \sum^\infty_{t=1} \mathbb{E}_{(\mathbf{s}_t, \mathbf{a}_t \sim \rho_\pi)}\bigg[\gamma^{t-1} \big( r(\mathbf{s}_t, \mathbf{a}_t) + \alpha \mathcal{H}(\mathbf{a}_t \mid \mathbf{s}_t) \big)\bigg], \tag{3}
$$

where $$\rho_\pi$$ denotes the state-action marginal induces by the policy $$\pi$$ and $$\alpha$$ is a coefficient that trades off the reward and the entropy regularization terms. The entropy regularization results in the following, energy-based Boltzmann distribution as optimal policy:

$$
\pi^*_{\text{MaxEnt}}( \mathbf{a}_t \mid \mathbf{s}_t)
	=
	\exp{
		(
		Q^*_{\text{soft}} (\mathbf{s}_t, \mathbf{a}_t) 
		-
		V^*_{\text{soft}} (\mathbf{s}_t) 
		),
	}
  \tag{4}
$$

with the optimal *soft* value and Q-function given by 

$$
V^*_\text{soft}(\mathbf{s}_t) 
	= 
	\log
	\int_\mathcal{A}
	\exp
	(
	Q^*_\text{soft}(\mathbf{s}_t, \mathbf{a}^\prime) 
	)
	\,
	d \mathbf{a}^\prime
	,
  \tag{5}
$$

and

$$
Q^*_\text{soft}(\mathbf{s}_t, \mathbf{a}_t) 
	= 
	r(\mathbf{s}_t, \mathbf{a}_t) 
  +
  \\
	\mathbb{E}_{(\mathbf{s}_{t+1}, \dots) \sim \rho_\pi} 
	\bigg[ 
	\sum_{l=1}^\infty 
	\gamma^l 
	\big(
	r(\mathbf{s}_{t+l}, \mathbf{a}_{t+l}) 
	+ 
	\mathcal{H}(\mathbf{a}_{t+l} \mid \mathbf{s}_{t+l})
	\big) 
	\bigg]
	.
  \tag{6}
$$

In Equation (4), the soft Q-function serves as negative energy for the Boltzmann distribution adn the soft value function serves as log partition function, which means that for sampling, we can ignore the value function and directly sample from the unnormalized density

$$
\pi^*_{\text{MaxEnt}}( \mathbf{a}_t \mid \mathbf{s}_t)
	\propto
	\exp{
		(
		Q^*_{\text{soft}} (\mathbf{s}_t, \mathbf{a}_t) 
		),
	}
\tag{7}
$$

for example with Monte Carlo methods or Importance Sampling.
For the rest of this post, we will drop the *soft* subscript to avoid visual clutter.
Importantly, Q-Decomposition is still possible in MaxEnt RL, meaning we can also decompose and reconstruct a soft, multi-objective Q-function as described before (we just a way to sample from the reconstructed, soft Q-function). 
We wil make use of [soft Q-Learning](http://proceedings.mlr.press/v70/haarnoja17a/haarnoja17a.pdf)(SQL) for continuous action-space MDPs to directly obtain $$Q^*_i$$, the optimal, soft constituent Q-functions for Q-Decomposition. 
We can then reconstruct the global, optimal, soft Q-function $$\bar{Q}^*$$and rely on Armortized Stein Variational Gradient Descent to learn a sampling network for it, which (but not exactly) corresponds to an actor-critic like algorithm.
Thus, in summary, we will use the SQL algorithm to learn constitutent Q-functions (and corresponding sampling networks) for some MORL problem. 
This then allows us to sum the constituent Q-functions to obtain the Q-function for the scalarized MORL problem, for which we also learn a sampling network to generate actions. 
In the next section, I describe the special kind of MORL problem that we are interest in solving.

<h2 id="lexi_morl">Lexicographic MORL</h2>
As mentioned before, we are interested in solving special, continuous state- and action-space MORL problems while exploiting the benefits associated with Q-Decomposition. 
In particular the *special* MORL problems we are interested in are refered to as *lexicographic* or *task-prioritized* MORL problems. 
These are problems where the MORL subtasks are ordered by priority, e.g. for a lexicographic MORL problem we might define that the subtask for obstacle avoidance is of higher priority than the subtask for navigating to some goal.
This makes for a natural and intutive way of specifying tasks. 
Arguably, its much more convenient and more intuitive to simply specify an order over subtasks than to carefully design a scalar-valued reward function that induces some complex behavior.
Formally, a lexicographic MORL problem is given a MDP $$\mathcal{M}_\succ \equiv (\mathcal{S, A}, \mathbf{r}, p, \gamma, o, \varepsilon)$$, where $$o = \langle 1, 2, \dots i \dots n \rangle$$ specifies the priority-order of subtasks and $$\varepsilon = \langle \varepsilon_1, \varepsilon_2, \dots \varepsilon_i \dots \varepsilon_n \rangle$$ are certain threshold variables. 

Informally speaking, solving a lexicographic MORL problem means finding a policy that is optimal for the highest priority subtask, while for each lower-priority subtask it is as good as possible and subject to the constraint of not worsening performance of any of the higher-priority subtasks.
More formally, this means that policy search for each subtask $$i$$ is constrained to a set $$\Pi_i$$, which contains only those policies that are also optimal for each higher priority task $${1, \dots,  i-1}$$.

We allow for some worsening of higher-priority subtask performance $$J$$, since otherwise the set $$\Pi_i$$ would only contain the optimal policies for the higher-priority subtasks. 
How much we allow performance of higher-priority subtasks to worsen is defined by the $\varepsilon_i$ threshold scalars, which means policy search for subtask $$i$$ is constrained to the set

$$
\Pi_{i} = \{ \pi \in \Pi_{i-1} \mid \underset{\pi' \in \Pi_{i-1}}{\max} J_{i-1}(\pi') - J_{i-1}(\pi) \le \varepsilon_{i-1} \}.
\tag{8}
$$

In practise, computing the set $$\Pi_i$$ is intractable for continuous state- action-space MDPs. For our method, we instead implement Equation (8) locally and state-based, as described in the next section.

<h2 id="psqd">Prioritized Soft Q-Decomposition (PSQD)</h2>
Our learning algorithm implements the lexicographic constraint from Equation (8) locally and state-based:

$$
\max_{\mathbf{a}^\prime \in \mathcal{A}} Q_i(\mathbf{s}, \mathbf{a}^\prime)
	-
	Q_i(\mathbf{s}, \mathbf{a})
	\leq \varepsilon_i,
  \\
	\forall  \mathbf{a} \sim \pi_\succ,
  \\
	\forall \mathbf{s} \in \mathcal{S},
  \\
	\forall i \in \{1, \dots, n - 1\}.
  \tag{9}
$$

Using this implementation of the task priority constraint, the agent, in every state, can only select ations whose expected, discounted, future return is 
worse that that of the optimal action, minus $$\varepsilon_i$$.
This formulation has an important effect: For every except the highest-priority (unconstrained) task $$i \ge 2$$, the action selection is constrainted to some subset of $$\mathcal{A}_\succ$$ of the original MDP's action space $$\mathcal{A}$$. In our paper, we refer to this subset as the *indifference space* of task $$i$$, since with respect to the constraint, the task is indifferent as to which action is executed. Thus, this formulation of the lexicographic constraints results in a binary mask over the action space 

Based on this formulation of the lexicographic constraint, we propose Prioritized Soft Q-Decomposition (PSQD), an iterative learning algorithm based on SQL and Q-Decomposition for lexicographic MORL problems. 


This 
From this simple formulation of the lexicographic constraint, 

This makes for unconstrained optimization --> different MDP. 

We can pre-train separately.

And then finetune in transformed MDP.

<h2 id="results">Results</h2>



<h2 id="summary">Summary and conclusion</h2>


And that's it, I hope you found this post interesting :)

Cheers,<br>
*Finn*.
<br>