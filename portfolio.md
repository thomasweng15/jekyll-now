---
layout: default
---

# Portfolio

This is a curated list of past projects in chronological order. 

## Deep Learning on a Drone to Track Targets in Simulation

![image](https://github.com/thomasweng15/RoboND-DeepLearning-Project/raw/master/screenshot1.png)

I trained an autoencoder network to recognize a target individual in simulation and follow the person around. I completed this project as part of the first term of the Udacity Robotics Nanodegree.

Read my full [writeup](https://github.com/thomasweng15/RoboND-DeepLearning-Project/blob/master/writeup.md) on the project, including the network architectures I tried and what ultimately worked, or view the [Github repository](https://github.com/thomasweng15/RoboND-DeepLearning-Project/blob/master/writeup.md).

_November 2016_

## Object Recognition with the PR2

![image](https://raw.githubusercontent.com/thomasweng15/RoboND-Perception-Project/master/images/world3objrec.png)

Using point cloud data from a PR2 sensor, I isolated and identified objects in a tabletop environment. I used RANSAC table segmentation to separate the table from the objects, and identified the objects using a trained Support Vector Machine on the color and normal histograms of the objects. 

I completed this project in simulation using resources from the Udacity Robotics Nanodegree, and implemented it on an actual PR2 soon afterwards for a research project at University of Washington. For more details on the project, see the [Github repository](https://github.com/thomasweng15/RoboND-Perception-Project).

_October 2016_

## Kinematics with the Kuka KR210

![img](https://raw.githubusercontent.com/thomasweng15/RoboND-Kinematics-Project/master/misc_images/misc2.png)

I performed a kinematic analysis of the Kuka KR210 to derive a Denavit-Hartenberg Table of parameters for forward and inverse kinematics. After the analysis, coded up the forward and inverse kinematics to complete a pick and place task in simulation. This project was completed as part of the Udacity Robotics Software Nanodegree. Read the [writeup](https://github.com/thomasweng15/RoboND-Kinematics-Project/blob/master/writeup.md) or view the [repo](https://github.com/thomasweng15/RoboND-Kinematics-Project). 

_September 2016_

## Building a mobile robot with the Raspberry Pi

<a data-flickr-embed="true"  href="https://www.flickr.com/photos/145491926@N08/31734024384/in/dateposted-public/" title="softwarediag"><img src="https://farm1.staticflickr.com/479/31734024384_09b6abdef1_k.jpg" width="2048" height="1536" alt="softwarediag"></a><script async src="//embedr.flickr.com/assets/client-code.js" charset="utf-8"></script>

I built a two-wheeled robot from scratch using the Raspberry Pi and Robot Operating System. I designed the wiring and wrote code to move the robot via an Xbox controller. I presented my work at Microsoft Robotics Day and wrote about the experience here: [Blog post on Microsoft Robotics Day](http://www.thomasweng.com/microsoft_robotics_day/)

I am in the process of integrating a 2D lidar on top of the robot so that it can run SLAM and follow people. Stay tuned for more on this. 

_January 2016_

## Senior Project: Realtime Gesture Perception

<div style="width:100%;height:0;padding-bottom:56%;position:relative;"><iframe src="https://giphy.com/embed/l0HUh6X7g8NCZl1Cw" width="100%" height="100%" style="position:absolute" frameBorder="0" class="giphy-embed" allowFullScreen></iframe></div>

I implemented a gesture perception system using the Microsoft Kinect 2 and connected it with a Nao robot. At the time, the Kinect 2 was recently released and was equipped with better sensors than the original, but there were no open-source Linux drivers for the Kinect 2. I used a websocket to send data between the Kinect running on Windows with the Linux-controlled Nao. The result was a realtime gesture recognition system that could detect gazes and points toward objects in a scene. 

Following this work, I wrote code to allow the Nao to look and point at objects as well. Here is the repository for that work: [Nao looking and pointing](https://github.com/thomasweng15/nao-looking-and-pointing)

These projects formed the basis for the system implementation and user study evaluation in two research papers: 
1. Admoni, H., Weng, T., and Scassellati, B. Modeling communicative behaviors for object references in human-robot interaction. In _IEEE International Conference on Robotics and Automation (ICRA)_, pages 3352-3359. _(Acceptance rate: 35%)_

2. Admoni, H., Weng, T., Hayes, B. and Scassellati, B. Robot nonverbal behavior improves task performance in difficult collaborations. In _ACM/IEEE International Conference on Human Robot Interaction (HRI)_, pages 51-58. {\color{gray} _(Acceptance rate: 25%)_

_April 2015_

## Boggle Solver
I wrote a C program to play [Boggle](https://en.wikipedia.org/wiki/Boggle) after being asked how I would do it in an interview. 

I coded all the data structures and algorithms from scratch, using a linked list with depth-first search to traverse the board, a hash set to keep track of which parts of the board I already visited, and a trie to represent and search the word dictionary. I learned a lot in this process and became more familiar with these structures and algorithms. 

I demonstrate how the solver works below. After inputting the grid of letters, the program returns a list of words that are valid for the board, which I then input into the game. 
<div style="width:100%;height:0;padding-bottom:63%;position:relative;"><iframe src="https://giphy.com/embed/3ohjUVRgcNmdcPd6i4" width="100%" height="100%" style="position:absolute" frameBorder="0" class="giphy-embed" allowFullScreen></iframe></div>

_December 2013_

