---
layout: post
title: Filter MoveIt cartesian path plans with jump_threshold
author: Thomas Weng
comments: true
tags: 
- how-to
- robotics
---

[MoveIt](https://moveit.ros.org/) is a useful tool for robot motion planning, but it often lacks documentation for key functions and features. One particularly opaque function is `compute_cartesian_path`, which takes as input waypoints of end effector poses, and outputs a joint trajectory that visits each pose. Although this function is in the [tutorials](http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html), the tutorials don't cover the critical role of the `jump_threshold` parameter when running the function on a real robot. 

On real robots, executing the trajectory produced by `compute_cartesian_path(..., jump_threshold=0)` can sometimes cause the robot to jerk unpredictably away from the desired path. This undesirable motion seems to occur when the arm approaches singularities and can't continue following the waypoints without reorienting the arm.[^1]

The solution to this problem is to set the `jump_threshold` parameter, which limits the maximum distance or "jump" in joint positions between two consecutive trajectory points. When this parameter is set, the planner returns a partial solution if consecutive joint positions exceed the threshold, so the robot will no longer move jerkily. 

But what should the value of `jump_threshold` be? Looking at the [code](
https://github.com/ros-planning/moveit/blob/701fbddb26f5aa4e752235b893292fc4618da135/moveit_core/robot_state/src/robot_state.cpp#L2096), the trajectory points must satisfy the following condition: 

    distance between consecutive joint positions <= jump_threshold * mean joint position distance

So the distance must be less than or equal to the threshold times the mean distance.[^2] In practice, I played with the threshold[^3] until the robot moved without jerking or stopping prematurely. 

Tuning the `jump_threshold` was critical for getting cartesian path planning working on our robot arm. Now, we plan the whole trajectory up front and abort if only a partial trajectory is found. Hope this helps!

---
Footnotes

[^1]: In most cases, the cartesian planner will recognize that there is no feasible plan and return a partial trajectory. But sometimes it seems to attempt to reorient the arm quickly, which causes the jerky deviation from the desired path. 
[^2]: There is [code](https://github.com/ros-planning/moveit/blob/701fbddb26f5aa4e752235b893292fc4618da135/moveit_core/robot_state/src/robot_state.cpp#L2132) for using an absolute distance condition instead of the mean. Also see this [Github issue](https://github.com/ros-planning/moveit/issues/773) for relevant discussion.
[^3]: I use `jump_threshold=5.0` but your mileage may vary. 
