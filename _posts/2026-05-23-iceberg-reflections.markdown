---
title: "Reflections on Project Iceberg and an AI Engel's Pause"
last_modified_at: 2026-05-23 12:23:31 +0200
categories:
  - Machine Learning
tags:
  - AI economy
mathjax: true
published: true
toc: true
toc_sticky: true
teaser: "Exploring MIT's Project Iceberg report, the realities of the AI economy, and the dawning possibility of an AI Engel's Pause."
---

## Introduction

The relentless pace of AI research and the accelerating AI adoption across industries have sparked fierce debates about the future of the global economy.
For some, the impact has already been felt, with entry-level positions in AI-exposed sectors, such as software engineering, experiencing a roughly 20% drop in hiring since the release of ChatGPT in late 2022 <a href="#ref1">[1]</a>.

While this downturn is devastating for those affected, 
I believe we are seeing only the tip of the iceberg. 
I believe we are far from experiencing the full economic impact of AI automation, and that the next few years will bring a wave of disruption that will reshape the labor market in profound ways.
This belief is based on my experience as an AI researcher and engineer.
I have built AI knowledge bases, harnesses, and agentic AI systems, and I have the strong belief that a significant percentage of white-collar work can be automated using a robust knowledge base and a decent deployment harness—aka with a good agentic AI system. 
Physical labor may not be far behind either, given the breathtaking achievements in real-world, field robotics recently, e.g., Figure's 200-hour non-stop package sorting stream with three humanoids <a href="#ref2">[2]</a>.

Consider how humans transmit knowledge: we onboard employees, explain workflows, and reason through problems using language. Because human cognition is structurally bound to language, the vast majority of our collective knowledge is stored and transferred as text. Today's AI systems excel at processing text, meaning they can ingest, and increasingly reason in, almost any domain of human expertise. 

This leads me to believe that we are approaching a threshold where any task a human can explain to another human can be engineered into an agentic AI system. The macroeconomic implications of such a shift are massive, which led me to read MIT's recent research to understand how economists are modeling this transition.


## Summary of Project Iceberg

MIT’s Project Iceberg <a href="#ref3">[3]</a> offers a population-scale framework designed to model how AI agents will impact the workforce. In their report, *The Iceberg Index* <a href="#ref4">[4]</a>, Chopra et al. distinguish between the immediately visible impacts of automation and its hidden potential, dividing them into two primary metrics:

The **Surface Index** measures easily quantifiable, present-day AI adoption. This index, essentially, accounts for the tangible impacts currently driving news headlines. According to the report, this visible automation already accounts for 2.2% of the national wage value in the United States, translating to roughly $200 billion in compensation for labor that can currently be handled by AI.

The **Iceberg Index** represents the underwater, hidden portion of potential AI automation on the labor market. The report calculates that 11% of US wage value is currently paid for labor that is technically automatable by AI capabilities, today. 

To determine this index, Chopra et al. broke jobs down into 32,000 distinct, low-level skills (e.g., "Programming," "Critical Thinking") and behaviors (e.g., "Analyzing Data," "Interacting with Computers"), assessing the density of these skills within the broader labor market.
Contrary to the already visible impacts on AI-exposed sectors captured by the Surface Index, the Iceberg Index reveals that a much larger portion of the labor market is technically vulnerable to AI automation, even if adoption has not yet happened at scale:

| Metric | Percentage of US Wage Value | Estimated Economic Value |
| :--- | :--- | :--- |
| **Surface Index** (Active Adoption) | 2.2% | ~$200 Billion |
| **Iceberg Index** (Technical Potential) | 11.0% | ~$1 Trillion |

Chopra et al. explicitly note that the Iceberg Index is strictly an assessment of AI capabilities and how these capabilities are distributed across the labor market; it is not meant as a forecast of job loss due to AI automation. 


## Commentary and Analysis

While the Project Iceberg report provides a valuable structural framework for understanding the potential reach of AI automation, it explicitly does not attempt to model the macroeconomic implications of this shift, and therefore leaves many questions unanswered.
I see the findings in this report as a confirmation of my concerns.
Not with some hypothetical future AGI, but already with AI capabilities that are available today, we can essentially automate a significant portion of the labor market.

While the actual percentage of job loss due to AI automation will heavily depend on the rate of adoption, policy, and regulatory responses, the fact remains that a significant portion of the labor market is technically vulnerable to automation.
What happens to a consumption-driven economy when $200 billion, and eventually $1 trillion, in wages stops flowing to human workers? 
Currently, human wages recycle directly back into local economies, paying for mortgages, groceries, restaurants, and local services. 
When an AI system replaces a human worker, it does not substitute their economic participation. 
AI systems do not buy goods, pay into social safety nets, or frequent local businesses. 
Economists frequently note that a single high-paying tech job creates and sustains five local service roles <a href="#ref5">[5]</a>, <a href="#ref6">[6]</a>.
If those tech jobs vanish, the five dependent service jobs are bound to follow, creating a cascading effect of economic disruption. 

Prominent figures at frontier AI companies seem to share concern surrounding the topic, as they have been vocal about the economic implications of AI automation in not-exactly confidence-instilling terms:

> "[We will have to] navigate through a difficult economic transition." -- **Sam Altman**, CEO, OpenAI <a href="#ref7">[7]</a>

> "[The AI revolution will have] 10 times the impact the Industrial Revolution had, but 10 times faster." -- **Demis Hassabis**, CEO, Google DeepMind <a href="#ref8">[8]</a>

> "Our current economic setup will no longer make sense." -- **Dario Amodei**, CEO, Anthropic <a href="#ref9">[9]</a>

### Avoiding an AI Engel's Pause
When attending the post-AGI workshop at ICLR 2026, I became aware of a historical economic phenomenon called the **Engel’s Pause** <a href="#ref10">[10]</a>. 
The Engel’s Pause describes a period during the early Industrial Revolution where British GDP grew rapidly, but working-class wages stagnated for decades, leaving the vast majority of the population initially worse off despite overall economic growth.

I fear that we may be heading toward a similar scenario with AI automation, and I am using my small platform here to help raise awareness about that distinct possibility. 
Admittedly, I am still working to fully grasp the full macroeconomic picture. I am an AI researcher by trade, unfortunately not an economist. 
However, I am on a mission to do whatever I can to help prepare for and soften the blow when the true scale of AI automation begins to hit the workforce. 
This blog post is a first step in that direction, and as I continue to learn and unpack this transition, I plan to share more of my findings here.

Best,
Finn.

---

## References

* <span id="ref1"> [1] [Canaries in the Coal Mine? Six Facts about the Recent Employment Effects of Artificial Intelligence](https://digitaleconomy.stanford.edu/publication/canaries-in-the-coal-mine-six-facts-about-the-recent-employment-effects-of-artificial-intelligence/) </span>
* <span id="ref2"> [2] [Figure’s robots just sorted packages for 200 hours straight](https://sherwood.news/tech/figures-robots-just-sorted-packages-for-200-hours-straight/) </span>
* <span id="ref3"> [3] [MIT Project Iceberg](https://iceberg.mit.edu/) </span>
* <span id="ref4"> [4] [The Iceberg Index: Measuring Skills-centered Exposure in the AI Economy](https://arxiv.org/pdf/2510.25137v1) </span>
* <span id="ref5"> [5] [The New Geography of Jobs, Enrico Moretti](https://moretti.econ.berkeley.edu/book) </span>
* <span id="ref6"> [6] [Multiplier Effects: Connecting the Innovation and Opportunity Agendas](https://www.brookings.edu/articles/multiplier-effects-connecting-the-innovation-and-opportunity-agendas/) </span>
* <span id="ref7"> [7] [Sam Altman Blog](https://blog.samaltman.com/) </span>
* <span id="ref8"> [8] [Lex Fridman Podcast: Demis Hassabis Interview](https://lexfridman.com/demis-hassabis-2-transcript/) </span>
* <span id="ref9"> [9] [Dario Amodei Essay: Machines of Loving Grace](https://www.darioamodei.com/essay/machines-of-loving-grace) </span>
* <span id="ref10"> [10] [Wikipedia: Engels' pause](https://en.wikipedia.org/wiki/Engels%27_pause) </span>