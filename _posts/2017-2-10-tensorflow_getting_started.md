---
layout: post
title: Getting Started with Tensorflow
---

I took some time out this past weekend to work through an introductory machine learning talk on building models using Tensorflow. Within a few hours, I coded up a model classifying handwritten digits from the MNIST dataset with 99.5% accuracy! Considering that the accuracy of cutting edge research models currently hover at around 99.7%, my model was a pretty good result for just a few hours of development.

The resource I relied on to ramp up was [Google's crash course through Tensorflow](), presented by [Martin Gorner]() at [Devoxx](). The talk starts with a simple one-layer neural network, building quickly into multi-layer neural nets, convolutional neural nets, recurrent neural nets, and a collection of optimization techniques (e.g. learning rate decay, dropout, batch normalization).

I took the time to write out the code and get things working on my machine. That took longer than simply watching the video, but it deepened my understanding and guaranteed that I didn't gloss over any important details. The slides from the talk included most of the code required for the models, but I did need to fill in some of the gaps. When I got stuck, I referenced the complete model at the [Github repo]() Gorner created for this talk.

Gorner's presentation glosses over the math behind concepts presented in order to focus on model building. I think this was a smart tradeoff. Having spent quite a bit of time taking courses and learning the math powering machine learning, the details of how the math works would probably be too involved for a short talk. And when it comes to actually building models, Tensorflow does the heavy lifting and abstracts the mathematical details away anyway.

Of course, the math does become more relevant as one continues working with ML. I'm sure I wouldn't have picked things up so quickly with Tensorflow had I not already studied the background concepts.

I'm looking forward to playing more with Tensorflow and learning about what's going on under the hood. I'll be keeping up with the math through online resources -- courses, textbooks, and hopefully papers. Most of all, I'm excited to take the training wheels off soon and start building my own models from scratch!

<br>
<hr>
