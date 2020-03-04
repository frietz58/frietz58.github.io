---
title: "Machine Learning fundamentals I: Understanding the Loss Landscape and the Gradient Descent algorithm"
categories:
  - Machine Learning
tags:
  - Machine Learning fundamentals
  - Loss function
  - Gradient Descent
mathjax: true
published: false
toc: true
toc_sticky: true
teaser: "In part one of the 'Machine Learning fundamentals' series, we explore the Loss Landscape and its association to the Gradient Descent algorithm."
---

<h2 id="motivation">Introduction and motivation</h2>
I remember the first weeks of attending <a href="https://sites.google.com/view/victoruccetina/" target="_blank">Professor Uc-Cetina's </a> lecture on Machine Learning, which was my first academic encounter with advanced Machine Learning principals and algorithms. While I learned a lot in the lecture, I remember struggeling quit heavily to grasp everthing, especcially in the beginning the course. While there are numerous guides, tutorials and write-ups that aim explaining the basic Machine Learning principals (e.g. Gradient Descent and Backpropagation), I felt like many of them don't go into depth, but simply throw the almighty <i>Chain rule</i> at the reader, and thats that... <br><br>
In this <i>Machine Learning fundamentals</i> series, I will take my own spin at the topic, considering the very aspects that I feel are often left out, but which are crucial for fully understanding how things come together. Thus, in this post, we will explore the loss landscape, and how its associated with the Gradient Descent algorithm.
<br>


<h2 id="supervised_learning">Setting the scene for supervised learning</h2>
For this series and for demonstration purposes, we will consider and restrict ourself to a problem from the <a href="https://en.wikipedia.org/wiki/Supervised_learning" target="_blank">supervised learning family</a>. In supervised learning, we are interessted in learning the estimate of a function that models the relationship between some input data and the resulting output data, where <i>learning</i> refers to the updating of some parameters, which control our output behavior, for all input data. Further, for supervised learning, this process is based on showing many combinations of input and <b>correct</b> output patterns to some learning algorithm, with idea being that the learning algorithms discovers all the similarities and characteristics in the training data, thus learning the relationship between input and output. <br>  

<b>But how does it work?</b><br>

Well, let's consider how learning works in the real world. Consider a dermatologist, whos job involves looking at abnormalities of skin cells. For every patient, the doctor has to decide whether the patient has a harmless birthmark or skin cancer. While I did not undergo the education of a dermatologist, for the sake of the analogy, we will assume that the training of the dermatologist involved looking at many pictures/examples of different kinds of abnormalities, where the doctor had to make initially uneducated guesses, regading the class of abnormality currently being investigated. After making a guess, the doctor would be told by a teachre whether the guess was correct, or what would be the actual correct class. We can assume that at the beginning of this studies, the doctor would likely make many mistakes due to a lack of experience, but by observing more and more examples, the doctor would likely learn specific characteristics of the input pictures being presented and the output of the class association of the abnormality (e.g. cancerous or harmless). <br>

Given that this form of learning, e.g. starting with no knowledger about a topic but gaining and understanding through the collection of experience, appears to work well for humans, it would be worthwile to try to translate this loop of making a guess and being corrected into the theoretical world, where algorithms reside and thrive. In fact, you might have guessed it, this is exactly what is being done when we walk about supervised learning. <br>

To translate the dermatologist example into a supervised learning problem and to introduce the the neccesarry vocabulary, consider the following: The learning algorithm for the supervised problem would be the dermatologist, which actually learns how to map from input data(e.g. images of skin abnormalities) to the desired output (e.g. what kind of abnormality the dermatologist/learning algorithm is currently facing). The examples that we show to the dermatologist/learning algorithm are called <i>trainineg examples</i> (because the algorithm learns/trains on those) and the collection of training examples is refered to as the <i>training set</i>
In principal, the input and output combination for a supervised learning problem can be arbitrary, but we must always consider whether what we are doing actually makes sense. We could try to predicit tomorrows whether from the image of the skin abnormality, but it is very likely that the information is simply <b>not present in the data</b>, in whice case even the best learner would not make any progress. This also nicely hints at <a href="https://www.theguardian.com/commentisfree/2018/mar/28/all-the-data-facebook-google-has-on-you-privacy" target="_blank">why data protection and privacy is nowdays more important then ever</a>.


<h2 id="loss_landscape">Introducing the loss function</h2>
As we have established, after making a guess regarding the output, we need some teacher that tells us how right or wrong we were with our guess/prediction. For machine learning algorithms, this is (in the broadest sense), the loss function. The loss function can, just as the combination of input data to output data, be relatively arbitrary. In principal, however, we want something that is proportional to how far off we were with our guess. Think about it like this: If we guess/predict that a cancerous abnormality is harmless, this would have dramatic consequences for the patient! In this case, we would want our loss function to produce a large loss value, which tells the learner that it just made a large or severe error. Equivalently, if the error of our guess was less severe, for example when we already got right that the abnormality is cancerous, but guessed a wrong family of cancer, the loss value rating should be not as large as for the previous case, because the error that we made was not as bad. <br>

In mathematical terms, the loss function could look something like this: $$MSE = {\frac{1}{n}\sum_{i=1}^n(y_i - \hat{y_i})^2}$$. Here, we sum over all our training examples ($$\sum_{i=1}^n$$), compare the actual output $$y_i$$ to our prediction/guess $$\hat{y_i}$$ and add the squared differences of that $$(\dots)^2$$ up. Finally, we normalize this score by deviding through $$n$$. So what does this give us? The average error our learning algorithm made over the entire training set (using this specific formula for calculating the loss). This is simply called the <b>Mean Squared Error</b> (MSE) over our training set and is a very popular loss function. <br>

Okay, at this point we almost got everything we need to start learning. We have our learning algorithm (the dermatologist) that makes guesses or predictions of the output for our input and the loss function (the teacher), that indicates how good or bad our guess was. 
But one crucial part is still missing: How do we actually learn, given the knowledge that about how good or bad our guess was?



<figure class="half">
    <a href="/assets/img/gradient-descent-loss/loss_landscape_example.jpg"><img src="/assets/img/gradient-descent-loss/loss_landscape_example.jpg"></a>
    <a href="/assets/img/gradient-descent-loss/train_val_loss_landscape.png"><img src="/assets/img/gradient-descent-loss/train_val_loss_landscape.png"></a>
    <figcaption>Caption describing these two images.</figcaption>
</figure>


This code produces the following plot:
<img src="/assets/img/advanced-list-sorting/unsorted_plot_bright.png">


Cheers,<br>
*Finn*.

<br>
P.S.: This post has been motivated by <a href="https://stackoverflow.com/a/59159270/10476976" target="_blank">my answer on stackoverflow</a> regarding the question <a href="https://stackoverflow.com/q/9764298/10476976" target="_blank">Is it possible to sort two lists(which reference each other) in the exact same way?</a>

<br>
References: <br>
<a href="https://en.wikipedia.org/wiki/Supervised_learning" target="_blank">[1] Supervised Learning</a><br>
<a href="https://www.theguardian.com/commentisfree/2018/mar/28/all-the-data-facebook-google-has-on-you-privacy">[2] All data Google and Facebook have about your </a><br>
<a href="https://wiki.python.org/moin/HowTo/Sorting#The_Old_Way_Using_Decorate-Sort-Undecorate" target="_blank">[2] Python: Sorting Mini-HOW TO</a><br>
<a href="https://www.programiz.com/python-programming/methods/built-in/sorted" target="_blank">[3] Python sorted()</a><br>
<a href="https://www.programiz.com/python-programming/methods/built-in/zip" target="_blank">[4] Python zip()</a><br>
<a href="https://medium.com/understand-the-python/understanding-the-asterisk-of-python-8b9daaa4a558" target="_blank">[5] Understanding the asterisk(*) of Python</a><br>
