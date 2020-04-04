---
title: "Machine Learning fundamentals I: An analogy"
categories:
  - Machine Learning
tags:
  - Machine Learning fundamentals
mathjax: true
published: true
toc: true
toc_sticky: true
teaser: "In part one of the 'Machine Learning fundamentals' series, we explore the Loss Landscape and its association to the Gradient Descent algorithm."
---

<h2 id="motivation">Introduction and motivation</h2>
I remember the first weeks of attending <a href="https://sites.google.com/view/victoruccetina/" target="_blank">Professor Uc-Cetina's</a> lecture on Machine Learning, 
which was my first "real" academic encounter with the principals and algorithms that enable what we refer to as Machine Learning. While I learned a lot in the lecture, 
I remember struggling quite heavily with grasping the underlying principals, especially at the beginning of the course. While there are numerous guides, tutorials, and write-ups 
that aim to explain the basic Machine Learning principals (e.g. Gradient Descent, Backpropagation etc.), I felt like many of them don’t go into depth, but simply throw the almighty 
<i>Chain rule</i> at the reader, without explaining what is <i>actually</i> going on... <br><br>
In this <i>Machine Learning fundamentals</i> series, I will take my spin at the topic, considering the very aspects that I feel are often left out, but which are crucial for fully 
understanding how things come together. Thus, in this first post of the series, we will begin super basic and form an analogy between learning in the real world and artificial learning 
in the mathematical world. <br>


<h2 id="supervised_learning">Setting the scene for supervised learning</h2>
For this series and for demonstration purposes, we will consider and restrict ourself to a problem from the 
<a href="https://en.wikipedia.org/wiki/Supervised_learning" target="_blank">supervised learning family</a>. In supervised learning, we are interested in learning the estimate of a 
function that models the relationship between some input data and the resulting output data. The, <i>learning</i> refers to the updating of some parameters, which control the output 
behavior of our function estimate, for all input data. Further, for supervised learning, this process is based on showing many combinations of input and <b>correct</b> output examples 
to some learning algorithm, with the idea being that the learning algorithms discover all the similarities and characteristics in the training data, thus learning the relationship between 
he input and output data. 
In a binary setting, this means that the learning algorithm learns what patterns or characteristics in the input relate to which output class. In theory, everything is said at this point.
 However, this might have left you with more questions than answers, so let me unpack what this actually means...<br>  

<b>So how does it work?</b><br>

Well, let's consider how learning works in the real world. Consider a dermatologist, whose job involves looking at abnormalities of human skin. For every patient, the doctor has to 
decide whether the patient he currently investigates has a harmless birthmark or, let's say, some form of skin cancer. While I did not undergo the education of a dermatologist, 
for the sake of the analogy, we will assume that the training of the dermatologist involved looking at many pictures/examples of different kinds of abnormalities, where the doctor had 
to make initially uneducated guesses regarding the class of abnormality currently being investigated. After making a guess, the doctor-in-training would be told by a teacher whether the guess was correct, or what would be the actual correct class. We can assume that at the beginning of the studies, the doctor would likely make many mistakes due to a lack of experience, but by observing more and more exemplary images (this is our input data), the doctor would likely learn specific characteristics of the input pictures and how they associate to the output, where the output is the guess/prediction regarding the type or class of abnormality. <br>

Given that this form of learning, e.g. starting with no knowledge about a topic but gaining an understanding through the collection of experience, appears to work well for humans, it would be worthwhile to try to translate this loop of making a guess and being corrected into the theoretical world, where mathematical algorithms reside and thrive. In fact, you might have guessed it, this is exactly what is being done when we walk about supervised learning. <br>

To translate the dermatologist example into a supervised learning problem and to introduce the necessary vocabulary, consider the following: The learning algorithm for the supervised problem would, in the real world, be the dermatologist, which actually <i>learns</i> how to map from input data (e.g. images of skin abnormalities) to the desired output (e.g. what kind of abnormality the dermatologist/learning algorithm is currently observing). The pictures that we show to the dermatologist/learning algorithm are called <i>trainineg examples</i> (because the algorithm learns/trains on those) and the complete collection of all training examples is referred to as the <i>training set</i>.
In principle, the input and output combination for a supervised learning problem can be arbitrary, but we must always consider whether what we are doing <b>actually makes sense</b>. 
We could try to predict tomorrow's weather from the image of the skin abnormality, but it is very likely that the information is simply <b>not present in the data</b>, 
in which case even the best learner would not make any progress. (With this in mind, you can figure out <a href="https://www.theguardian.com/commentisfree/2018/mar/28/all-the-data-facebook-google-has-on-you-privacy" target="_blank">why data-protection is so important nowdays</a>...)

<h2 id="loss_function">Telling right from wrong</h2>
After making a guess regarding the output, we need some teacher that tells us how right or wrong we were with our guess or prediction. For machine learning algorithms, this is (in the broadest sense), the loss function. The loss function can, just as the combination of input data to output data, be relatively arbitrary. In principle, however, we want the loss function to give a value that is proportional to how far off we were with our guess. Think about it like this: If we guess/predict that a cancerous abnormality is harmless, this would have dramatic consequences for the patient! In this case, we would want our loss function to produce a large loss value, which tells the learner that it just made a large or severe error. 
Equivalently, if the error of our guess was less severe, for example when we already got right that the abnormality is cancerous, but guessed a wrong family of cancer, the loss value rating should be not as large as for the previous case, because the error that we made was not as bad. And this is what the loss function does, so the loss function for a Machine learning algorithm is like the teacher for the real-world dermatologist in-training.
<br>

In mathematical terms, the loss function could look something like this: 
$$L = (y_i - \hat{y_i})^2$$, where $$y_i$$ is the <i>actual</i> output value (the one that the teacher has written down) and $$\hat{y_i}$$ is the one our learning algorithm produced. Thus, what's happening there is that we simply compare the predicted value (the guess the dermatologist made) versus the one that would have been the ideal value. Intuitively, we see that the value in the parenthesis will be larger when the difference between the two values is larger, which was the key requirement that we had for our loss function. The $$i$$ basis here is simply an index, indicating which training example we are currently investigating, and we square the value so that it is independent of the sign of the output. <br>
In practice, we are interested in the performance of our algorithm for every example in the training set, not just one. Thus, we use the following formula, which does the same thing as the one we just introduced, except that it applies and sums the value over the entire data set:  
$$MSE = {\frac{1}{n}\sum_{i=1}^n(y_i - \hat{y_i})^2}$$. Here, we sum over all our training examples ($$\sum_{i=1}^n$$), compare the actual output $$y_i$$ to our prediction or guess $$\hat{y_i}$$ and add the squared differences of that $$(\dots)^2$$ up. Finally, we normalize this score by deviding through $$n$$. Thus, we obtain average error our learning algorithm made over the entire training set (using this specific formula for calculating the loss). This is called the <b>Mean Squared Error</b> (MSE) over our training set and is a very popular loss function. <br>

Okay, at this point we almost got everything we need to actually start learning. We have our learning algorithm (the dermatologist) that makes guesses or predictions of the output for our input and the loss function (the teacher), which indicates how good or bad our guess was. 
But one crucial part is still missing: How do we actually learn, given that we know how large (or small) our error was?

<h2 id="parameters">About parameters and artificial learning</h2>
After having established how to rate the performance of our learning algorithm, we now need to figure out what to do with this information about how good our guesses are. In machine learning, learning manifests on the parameters of the learning algorithm. What exactly these parameters are depends on the specific learning algorithm, but for an artificial neural network, the parameters would be the interneural connections and their associated weights. More general, the parameters of our learning algorithm govern how we map from input to output, independently of the specific learning algorithm. <br>

Let's once more return to our dermatologist analogy, because one could make the argument that human learning also involves parameters, specifically, the updating of these. 
We said that the parameters of the learning algorithm control how we map from input to output. Thus, for our dermatologist, the parameters would model
specific characteristics in the image that the dermatologist observes. One such characteristic could be, for example, the presence of a specific pattern of a specific color, 
where the dermatologist learned, through repeated guessing and being corrected, that whenever he observed this pattern, the abnormality class corresponded to "cancerous". 
Or, said differently, we could say that the dermatologist learned that the presense of this characteristic increases the likelihood of an abnormality to be cancerous. 
The dermatologist would, however, not only rely on one pattern (in machine learning lingo, one such pattern would be called a <i>feature</i>) for the final prediction 
regarding the output but have identified many different characteristics in all the images that he observed. 
The dermatologist would use the totality of all these characteristics or features (meaning the overall absence or presence of all identified features) to make the final prediction. For each characteristic, the dermatologist would have <i>learned</i> 
how its absence or presence relates to the class of the abnormality, and whether each feature is particularly strong or thrustworthy for the final classification. 
If you think about the dermatologist as a function, one could be the distribution of color values in the picture, where the dermatologist learned how the output class relates to the 
distribution. 
<br>

The entire idea behind Machine Learning is to <b>not tell</b> the learning algorithm what we humans have already figured out (e.g. don't tell the algorithm that color could be a good indicator for the class of the abnormality), but let the algorithm figure this out on his own (e.g. let the algorithm <i>learn</i> the parameters of the mapping function).
<br>

Thus, to break the previous two paragraphs down, we can say that artificial learning <i>actually means</i> how we update the parameter of our learning algorithm for a specific problem, through repeatedly guessing the output for the presented input, checking how correct our guess was and taking the according steps to change our parameters. (How we actually update the parameters of our learning algorithm will be the topic for the next post in this series.)
<br>

More formally speaking, we wan't to change the parameters of our learning algorithm in such a way, that it results in less overall loss (we wan't to <i>minimize the loss</i>). Recall the MSE loss introduced earlier, that meassure the average loss over all our training data, where <i>loss</i> was the measurement for how wrong our guess for the output was. Thus, <i>minimizing the loss</i> simply means that we try to update our parameters in such a way, the the loss gets reduced, which indicates that the values that we predict are now closer to the actual value that we wanted to get. Minimizing the loss actually is equivalent to our dermatologist making fewer or less severe errors in the guessing/predicting of the class of the skin abnormality. 
<br> 

Originally, I wanted to explain at this point how the <i>artifical learning</i> actually works, mathematically speaking. But I have decided to end this post here. If all of the above made sense to you (and maybe you are already a bit familiar with Machine Learning), you can go right ahead a read part two of this series. However, if this is the first time you try to understand Machine Learning, I encourage you to pause and ponder for a while (maybe a day), really think about what you read and return to the second part of the series whenever you are ready. Even though nothing fancy happened in this post, Machine Learning is a huge topic and difficult to get started with, and everybody needs some time to get familiar with it. 

<h2 id="summary">Summary</h2>
To summarize this post, the key point that I try to communicate here is the following: In Machine Learning, specifically supervised learning, we want to find a function that maps from some
input data to some output, based on showing many training examples to the learning algorithm, that contain the input data and the desired output. How the function maps from input to output is controlled by the parameters of the function, thus, the process of artificial learning involves finding the parameters that produce the best results, which is measured by the overall loss over
all the training examples.


All the best,<br>
*Finn*.

<br>
References: <br>
<a href="https://sites.google.com/view/victoruccetina/" target="_blank">[1] Professor Uc-Cetina</a><br>
<a href="https://www.theguardian.com/commentisfree/2018/mar/28/all-the-data-facebook-google-has-on-you-privacy">[2] All data Google and Facebook have about your </a><br>
<a href="https://en.wikipedia.org/wiki/Supervised_learning" target="_blank">[2] Wikipedia Supervised Learning</a><br>
