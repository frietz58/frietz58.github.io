---
title: "Fun with Cellular Automata"
published: true
toc: true
toc_sticky: true
author_profile: true
teaser: >-
  Some information regarding the animation playing as the banner on my website's landing page.
header:
  image:  /assets/img/rpsls.gif
  teaser:  /assets/img/rpsls.gif
  custom_style: "max-width: none; width:auto; height: 100%;"
---
<br>

This page provides some information about the animation playing in the header of my website's landing page.

## What are Cellular Automata?
The animation that is playing on this website's landing page is a "Rock-Paper-Scissors-Lizard-Spock" Cellular Automaton. What is a cellular automaton? Cellular automata are simple dynamical systems in which complex behaviour tends to emerge
based on very simple update rules. Cellular automata typically employ a grid-like state and update each cell according to a neighbourhood function that indexes the grid and updates the current cell according to a specific update rule, operating on the values indexed by the neighborhood function. The neat thing about cellular automata is that we can obtain very interesting and different dynamic systems based on very simple update rules. We only need to implement a base class for the CA that holds the state, while each inheriting CA simply implements the update rule for the desired result.

##  Examples
The following animations are examples of cellular automata. The examples differ only in their update rules, which are just a few lines of code (additionally, I used different colour palettes for the different CAs, but this is a purely visual change). The animations are obtained by repeatedly applying the update rule to the initial state, which reveals how the state evolves over time.

### Rock-Paper-Sissors
In this CA, each cell is updated by playing rock-paper-scissors against its neighbours. If the current type is e.g. rock, the cell changes to paper if there are more than `n` paper cells in its neighbourhood. Otherwise, it stays the same. The "Rock-Paper-Scissor-Lizard-Sprock" cellular
automaton extends this idea and simply introduces two additional elements, as described [here](https://softologyblog.wordpress.com/2018/03/23/rock-paper-scissors-cellular-automata/), but the principle update rule remains the same. We get the following result, each colour corresponds to one of the five types:
<br>
<div style="text-align: center;">
<img class="align-center" src="/assets/ca/rpsls.gif">
<figcaption>Rock-Paper-Scissor-Lizard-Spock CA.</figcaption>
</div>
<br>

### Conway's Game of Life
[Conway's Game of Life](https://en.wikipedia.org/wiki/Conway%27s_Game_of_Life) is probably the most popular CA, in which cells can either be dead or alive, while the density of living cells determines whether cells stay alive, die, or spawn, of course depending only on the local neighbourhood. These dynamics roughly model overpopulation, underpopulation and reproduction. Conway's Game of Life is also well-known for various *patterns* that behave in certain ways under the game-of-life update rules. Examples include the glider (travels diagonally over the canvas), oscillators (oscillate between states), or canons (produce projectiles that travel over the canvas). Here's an example of Game of Life, again based on only proving the update rule for our base CA class:
<br>
<div style="text-align: center;">
<img class="align-center" src="/assets/ca/gol.gif">
<figcaption>Game of life with three gliders, one canon, and oscillators in each corner.</figcaption>
</div>
<br>

### Forest Fire Simulation
As the last example, CAs can also be used as a very simplistic simulation of fire spreading through a forest. The updates rule simply states that cells with neighbour cells on fire also catch fire, while empty cells have a slight chance of regrowing. Lightning strikes have a very low chance to occur and are the initial causes of fires. Based on these dynamics, we get the following result:
<br>
<div style="text-align: center;">
<img class="align-center" src="/assets/ca/fire.gif">
<figcaption>Forest fire simulation.</figcaption>
</div>
<br>

I doubt that a CA model like this would actually be used in the real world to simulate the spread of forest fires, as it is just too simplistic. However, CAs find some use in video games as particle physics models, as explained in [this cool video](https://youtu.be/VLZjd_Y1gJ8). Additionally, can be used for pseudo-random number generation (see [here](https://en.wikipedia.org/wiki/Rule_30)). Independently of their real-world usefulness, CAs are fun to play around with for an afternoon and can generate cool animations, as the above examples illustrate :)

### Code
My code for the above animations is available [here](https://gist.github.com/frietz58/239c84e2c24513138ed7dd5ae17a15cb). As mentioned, it can easily be modified to implement other types of CAs.
