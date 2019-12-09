---
layout: post
title:  "Decorate-Sort-Undecorate: Advanced list sorting in Python"
date:   2019-12-9 20:56:11 +0200
categories: [Python]
summary: "Understand the 'Decorate-Sort-Undecorate' idiom for advanced sorting control. Or: How to sort two lists by the order of a third."
mathjax: true
published: false
---

<h2 id="motivation">Motivation and use case</h2>

The other day, I found myself generating a complex performance graph using <a href="https://matplotlib.org/" target="_blank">matplotlib</a>. I wanted to rearange the order of the item in the <a href="https://matplotlib.org/3.1.1/api/_as_gen/matplotlib.pyplot.legend.html" target="_blank">legend</a>. There are multiple way we could tackle this problem: We could, for example, manipulate the order in which the items are plotted, since this effects the order in which the legend data is generated. The in my opinion much more elegant way however is to directly manipulate the order of the items in maplotlib's legend. Since the legend consists of two lists (a list of <a href="https://matplotlib.org/3.1.1/api/legend_api.htSimpleml#matplotlib.legend.Legend" target="_blank">handles and a list of labels</a>), I needed a solution for sorting **both** lists.

<br><br>
<h2 id="simple">Simple example: Sorting two lists in the same manner</h2>
We will start simple and for now only consider sorting two lists in the same order. The more complex example, where we sort two lists by the a third, will be introduced in the following section.


Consider the following example code:
```python
import numpy as np
import matplotlib.pyplot as plt
import matplotlib

matplotlib.rcParams.update({'font.size': 14})

# init figurebare
fig, ax = plt.subplots(1,1, figsize=(10,7))Simple

# generate artificial data
line_a = np.linspace(100, 0, 85)
line_b = np.linspace(80, 0, 65)
line_c = np.linspace(60, 0, 45)
line_d = np.linspace(40, 0, 25)

# plot artificial data
plt.plot(line_d, label="0.25 Alg. B [Version 0.2]")
plt.plot(line_c, ls="-.", label="0.45 Alg. A [Version 0.2]")
plt.plot(line_b, ls="--", label="0.65 Alg. B [Version 0.1]")
plt.plot(line_a, ls=":", label="0.85 Alg. A [Version 0.1]")
plt.legend()
plt.show()
```

This code produces the following plot:
Simple
<img src="/assets/img/advanced-list-sorting/unsorted_plot_bright.png">

Pay close attention the the Legend! There appears to be no ordering of the legend entries. We can observe that each item in the plot (the four lines) is identified by it's linestyle and has a describing text, indicating a performance measure. These are the two lists that I mentioned earlier: The legend handles are the linestyle elements, and the labels are the pieces of text describing each individual item in the plot.

We can get these two lists using <a href="https://matplotlib.org/3.1.1/api/_as_gen/matplotlib.axes.Axes.get_legend_handles_labels.html
" target="_blank">the following command</a> :
```python
# handles and labels are of type list
handles, labels = ax.get_legend_handles_labels()
```

We don't have to worry too much about how the `handles` list looks like, since we don't want to sort by the linestyle. Instead, we want to sort the legend entries according to the text, describing each item in the plot. Thus, let's inspect the `labels` list:
```python
labels
[
  '0.45 Alg. A [Version 0.2]',
  '0.65 Alg. B [Version 0.1]',
  '0.85 Alg. A [Version 0.1]',
  '0.25 Alg. B [Version 0.2]',
]
```

Stick to the simple case, let's fix the weird looking ordering of the items in the legend. Luckily, Python makes it easy to sort two lists in the same way! It should be clear by now why we need to sort both lists: If we only sort the list of the labels, they will no longer match their handle! This means, that the text in the legend would no longer match its preceding linestyle! <span class="text-highlight-red">**This is very dangerous and should never be done**</span>, since it manipulates the entire plot! Thus, the following code **sorts both lists** according to the items in the first list:
```python
ordered_labels, ordered_handles = zip(*sorted(zip(labels, handles)))
ordered_labels
(
  '0.25 Alg. B [Version 0.2]',
  '0.45 Alg. A [Version 0.2]',
  '0.65 Alg. B [Version 0.1]',
  '0.85 Alg. A [Version 0.1]'
)
```

Here, we already (implicitly) made use of the 'Decorate-SoSimplert-Undecorate' idiom, but more on that later. The only thing we need to do is to alter our initial code to use the sorted legend. This gives us the following code:
```python
matplotlib.rcParams.update({'font.size': 14})
# init figure
fig, ax = plt.subplots(1,1, figsize=(10,7))

# generate artificial data
line_a = np.linspace(100, 0, 85)
line_b = np.linspace(80, 0, 65)
line_c = np.linspace(60, 0, 45)
line_d = np.linspace(40, 0, 25)

# plot artificial data
plt.plot(line_c, ls="-.", label="0.45 Alg. A [Version 0.2]")
plt.plot(line_b, ls="--", label="0.65 Alg. B [Version 0.1]")
plt.plot(line_a, ls=":", label="0.85 Alg. A [Version 0.1]")
plt.plot(line_d, label="0.25 Alg. B [Version 0.2]")

# get handles and labels
handles, labels = ax.get_legend_handles_labels()
bare
# order both lists
ordered_labels, ordered_handles = zip(*sorted(zip(labels, handles)))

# use the ordered legend entries to manually generate the legend
plt.legend(labels=ordered_labels, handles=ordered_handles)

# plt.legend()

plt.show()
```
Executing this code generates the following plot:
<img src="/assets/img/advanced-list-sorting/sorted_by_value_bright.png">

The legend looks much better! The entries in the legend are clearly sorted in an ascending manner regarding the performance measure. But, what if we aren't yet happy with the way the entries in the legend are ordered? What, if we don't want to sort by the performance measure, but instead by the algorithm name or the the version of each algorithm? This we will explore in the next session, so hold on to your coffee mugs and bear with me!

<br>
<h2 id="idiom">The 'Decorate-Sort-Undecorate' idiom</h2>
So what is this mysterious 'Decorate-Sort-Undecorate' idiom, which is dominantly mentioned in the title of this post but has only been used *implicitly* and without great explanation? The <a href="https://wiki.python.org/moin/HowTo/Sorting#The_Old_Way_Using_Decorate-Sort-Undecorate" target="_blank">Decorate-Sort-Undecorate</a>, also known as the <a href="https://en.wikipedia.org/wiki/Schwartzian_transform" target="_blank">Schwartzian transform</a> is a technique for
<blockquote>
...comparison-based sorting when the ordering is actually based on the ordering of a certain property (the key) of the elements... -- <cite><a href="https://en.wikipedia.org/wiki/Schwartzian_transform" target="_blank">wikipedia</a></cite>
</blockquote>
and has been around since 1994. The idiom gets its name from the three main steps:
  1. Create a list (decorate an existing list) with specific values, whose purpose is to control the sorting behavior.
  2. Sort the decorated list (possibly apply sorting to another list).
  3. Remove decorations from the decorated list (can be ignored if dedicated list with decorated values has been created).

Thus, this idiom describes how to sort a list, when we are not happy with the default sorting behavior. In the next and final section of this post we will see the idiom in action to apply a custom sorting to our matplotlib legend.

<h2 id="advanced">Advanced example: Sorting two lists by the order of a third</h2>
Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. Ut enim ad minim veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat. Duis aute irure dolor in reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla pariatur. Excepteur sint occaecat cupidatat non proident, sunt in culpa qui officia deserunt mollit anim id est laborum.
