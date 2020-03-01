---
title: "Decorate-Sort-Undecorate: Advanced list sorting in Python"
last_modified_at: 2019-12-9 23:07:42 +0200
categories:
  - Python
tags:
  - Python
  - Matplotlib
  - Sorting
mathjax: true
published: true
toc: true
toc_sticky: true
teaser: "Understand the 'Decorate-Sort-Undecorate' idiom for advanced sorting in python"
---

<h2 id="motivation">Motivation and use case</h2>

The other day, I found myself generating a complex performance graph using <a href="https://matplotlib.org/" target="_blank">matplotlib</a>. I wanted to rearrange the order of the item in the <a href="https://matplotlib.org/3.1.1/api/_as_gen/matplotlib.pyplot.legend.html" target="_blank">legend</a>. There are multiple ways we could tackle this problem: We could, for example, manipulate the order in which the items are plotted since this effects the order in which the legend data is generated. The, in my opinion, much more elegant way is to directly manipulate the order of the items in matplotlib's legend. Since the legend consists of two lists (<a href="https://matplotlib.org/3.1.1/api/legend_api.htSimpleml#matplotlib.legend.Legend" target="_blank">a list of handles and a list of labels</a>), I needed a solution for sorting **both** lists.

<br>
<h2 id="simple">Simple example: Sorting two lists in the same manner</h2>
We will start simple and, for now, only consider sorting two lists (that indirectly reference each other) in the same order. The more complex example, where we sort two lists by the order of a third list, will be introduced in the following section.


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

Pay close attention to the legend! There appears to be no ordering of the legend entries. We can observe that each item in the plot (the four lines) is identified by it's linestyle and has a describing text, indicating a performance measure. These are the two lists that I mentioned earlier: The legend handles are the linestyle elements, and the labels are the pieces of text describing each item in the plot.

We can get these two lists using <a href="https://matplotlib.org/3.1.1/api/_as_gen/matplotlib.axes.Axes.get_legend_handles_labels.html
" target="_blank">the following command</a> :
```python
# handles and labels are of type list
handles, labels = ax.get_legend_handles_labels()
```

We don't have to worry too much about how the `handles` list looks like since we don't want to sort by the linestyle. Instead, we want to sort the legend entries according to the text, describing each item in the plot. Thus, let's inspect the `labels` list:
```python
labels
['0.45 Alg. A [Version 0.2]',
'0.65 Alg. B [Version 0.1]',
'0.85 Alg. A [Version 0.1]',
'0.25 Alg. B [Version 0.2]',]
```

Sticking to the simple case, let's fix the weird-looking ordering of the items in the legend. Luckily, Python makes it easy to sort two lists in the same way! It should be clear by now why we need to sort both lists: If we only sort the list of the labels, they will no longer match their handle! This means, that the text in the legend would no longer match its preceding linestyle! <span class="text-highlight-red">**This is very dangerous and should never be done**</span>, since it manipulates the entire plot! Thus, the following code **sorts both lists** according to the items in the first list:
```python
ordered_labels, ordered_handles = zip(*sorted(zip(labels, handles)))
ordered_labels
('0.25 Alg. B [Version 0.2]',
'0.45 Alg. A [Version 0.2]',
'0.65 Alg. B [Version 0.1]',
'0.85 Alg. A [Version 0.1]')
```

Here, we already (implicitly) made use of the 'Decorate-Sort-Undecorate' idiom, but more on that later. Let's break the `ordered_labels, ordered_handles = zip(*sorted(zip(labels, handles)))` line down into smaller pieces, to really understand what's happening. The most inner `zip(labels, handles)` shouldn't be too mysterious. The `zip()` function simply does what it always does: Creating an iterator of n-tuples (meaning there are n elements in the tuple), depending on how many iterable we pass into `zip()` function. To give an example:
```python
for tuple in zip(["fgh", "asd"], ["456", "123"]):
    print(tuple)
('fgh', '456')
('asd', '123')
```
In our case, this generates an iterator where each tuple contains one element of both lists, meaning the i-th tuple contains the i-th legend handle and i-th legend label. So far, so good, but what does the `*sorted(...)` do? Well, sorted simply sorts the items of a given iterable (the iterable we get from the most inner `zip()`.
```python
sorted(zip(["fgh", "asd"], ["456", "123"]))
[('fgh', '456'), ('asd', '123')]
```
Now, we get a list of sorted tuples. But we want two sorted lists! Here, the asterisk `*` comes into play. The asterisk, in this case, simply unpacks the list it receives as input into its positional arguments.
```python
print(*sorted(zip(["fgh", "asd"], ["456", "123"])))
('asd', '123') ('fgh', '456')
```
So now we no longer have a list of tuples, but rather an *iterable* of tuples. Remember what we could do with an iterable? Throw the iterable at `zip()` and get an iterable of n-tuples back! See where this is going? Look at the following snippet:
```python
for tuple in zip(*sorted(zip(["fgh", "asd"], ["456", "123"]))):
    print(tuple)
('asd', 'fgh')
('123', '456')
```
This is exactly what we wanted! Both lists have been sorted according to the items in the first list (actually, these are now tuples and not lists, but you can cast them into a list if it matters in your use case). Applying this to the initial snipped produces the following plot (if you want to take a look the complete, update snipped <a href="https://gist.github.com/frietz58/3453d0f421b08f0db68ec82ccfa497ec" target="_blank">Decorate-Sort-Undecorate</a>, also known as the <a href="https://en.wikipedia.org/wiki/Schwartzian_transform" target="_blank">here is the Github gist</a>):

Here is the resulting plot, in which the legend entries are sorted by the performance measure value:
<img src="/assets/img/advanced-list-sorting/sorted_by_value_bright.png">

The legend looks much better! The entries in the legend are sorted in an ascending manner regarding the performance measure. But, what if we aren't yet happy with the way the entries in the legend are ordered? What, if we don't want to sort by the performance measure, but instead by the algorithm name or the version of each algorithm? This, we will explore in the next two sections, so hold on to your coffee mugs and bear with me!

<br>
<h2 id="idiom">The 'Decorate-Sort-Undecorate' idiom</h2>
So what is this mysterious 'Decorate-Sort-Undecorate' idiom, which is dominantly mentioned in the title of this post but has only been used *implicitly* and without great explanation? The <a href="https://wiki.python.org/moin/HowTo/Sorting#The_Old_Way_Using_Decorate-Sort-Undecorate" target="_blank">Decorate-Sort-Undecorate</a>, also known as the <a href="https://en.wikipedia.org/wiki/Schwartzian_transform" target="_blank">Schwartzian transform</a> is a technique for
<blockquote>
...comparison-based sorting when the ordering is actually based on the ordering of a certain property (the key) of the elements... <cite><a href="https://en.wikipedia.org/wiki/Schwartzian_transform" target="_blank">wikipedia</a></cite>
</blockquote>
and has been around since 1994. The idiom gets its name from the three main steps:
  1. Create a list (decorate an existing list) with specific values, whose purpose is to control the sorting behavior.
  2. Sort the decorated list (possibly apply sorting to another list).
  3. Remove decorations from the decorated list (can be ignored if dedicated list with decorated values has been created).

Thus, this idiom describes how to sort a list, when we are not happy with the default sorting behavior. In the next and final section of this post, we will see the idiom in action to apply a custom sorting to our matplotlib legend.

<h2 id="advanced">Advanced example: Sorting two lists by the order of a third</h2>
Now, towards the more general example. Say we have two lists, like the list of handles and labels in a matplotlib legend, and we want to sort both of those lists according to some custom sorting behavior. Here, we fully embrace the *decorate* part of the 'Decorate-Sort-Undecorate' idiom by creating an additional list, with the sole purpose of generating the sorting behavior for our two *actual* lists.

For this, we must first generate the *decorated* list (the original idiom manipulates the actual list, but I feel like doing this externally is much more intuitive). As mentioned above, let's say we want to sort our legend entries not by the performance measure, but by the name of the algorithms;
```python
decorated = [text[5:] for text in labels]
decorate
['Alg. A [Version 0.2]',
'Alg. B [Version 0.1]',
'Alg. A [Version 0.1]',
'Alg. B [Version 0.2]']
```

In the next step, we generate an index list, and sort the indices according to the decorated list.
```python
sorted_indices = list(range(len(decorated)))
sorted_indices.sort(key=decorated.__getitem__)
sorted_indices
[2, 0, 1, 3]
```

The final step is to simply map the sorted indices to the two lists we want to sort and we are done!
```python
sorted_labels = list(map(labels.__getitem__, sorted_indices))
sorted_handels = list(map(handels.__getitem__, sorted_indices))
```

<br>
And that's it! Here's our final plot:
<img src="/assets/img/advanced-list-sorting/sorted_by_name_bright.png">

We have applied the 'Decorate-Sort-Undecorate' idiom to sort two lists in the same manner. The final version of the initial snipped, which contains the above steps is available as <a href="https://gist.github.com/frietz58/e7f3b0b4590ddb36a2dcc926644a46a3" target="_blank">gist on Github</a>. I hope you learned something from this post, or if not, at least enjoyed reading it as much as I enjoyed writing it.

Cheers,<br>
*Finn*.

<br>
P.S.: This post has been motivated by <a href="https://stackoverflow.com/a/59159270/10476976" target="_blank">my answer on stackoverflow</a> regarding the question <a href="https://stackoverflow.com/q/9764298/10476976" target="_blank">Is it possible to sort two lists(which reference each other) in the exact same way?</a>

<br>
References: <br>
<a href="https://en.wikipedia.org/wiki/Schwartzian_transform" target="_blank">[1] Schwartzian transform</a><br>
<a href="https://wiki.python.org/moin/HowTo/Sorting#The_Old_Way_Using_Decorate-Sort-Undecorate" target="_blank">[2] Python: Sorting Mini-HOW TO</a><br>
<a href="https://www.programiz.com/python-programming/methods/built-in/sorted" target="_blank">[3] Python sorted()</a><br>
<a href="https://www.programiz.com/python-programming/methods/built-in/zip" target="_blank">[4] Python zip()</a><br>
<a href="https://medium.com/understand-the-python/understanding-the-asterisk-of-python-8b9daaa4a558" target="_blank">[5] Understanding the asterisk(*) of Python</a><br>
