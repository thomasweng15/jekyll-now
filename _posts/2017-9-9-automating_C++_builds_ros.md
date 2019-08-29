---
layout: post
title: Automating C++ builds in ROS
author: Thomas Weng
comments: true
tags:
- productivity
- robotics
minutes: 3
---

If you're working on a C++ ROS project, you probably run `catkin build` every time you make a change. This is tedious and takes you out of your programming flow. It's especially annoying when your build fails multiple times due to small errors. I'm a big proponent of keeping the iteration loop as small as possible.[^1]

To fix this, I've automated the build process to build when saving a file! No more manual building :).

Here's how it works. I've written a shell script called `builder.sh` that kicks off a build for you every time you save a file in your source directories. If the build fails, it will output the error. If the build succeeds, it'll print a success message. Here's an example of a build failure, followed by success:

```
$ bash builder.sh
Setting up watches.
Watches established.
octomapper.cpp modified, rebuilding...
_______________________________________________________________________________
Errors     << perception:make /home/tweng/catkin_ws/logs/perception/build.make.1447.log
/home/tweng/catkin_ws/src/ros-project/src/perception/src/octomapper.cpp: In member function ‘void perception::Octomapper::publish_octomap(octomap::OcTree*)’:
/home/tweng/catkin_ws/src/ros-project/src/perception/src/octomapper.cpp:99:3: error: ‘pub’ was not declared in this scope
   pub.publish(octomap_msg);
   ^
make[2]: *** [CMakeFiles/perception_octomapper.dir/src/octomapper.cpp.o] Error 1
make[1]: *** [CMakeFiles/perception_octomapper.dir/all] Error 2
make: *** [all] Error 2
cd /home/tweng/catkin_ws/build/perception; catkin build --get-env perception | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
...............................................................................
Failed     << perception:make           [ Exited with code 2 ]
```
There is syntax highlighting in your terminal which makes this output more readable.

After fixing the issue and saving, the build runs again automatically:
```
octomapper.cpp modified, rebuilding...
[build] Summary: All 2 packages succeeded!
```

It's fairly simple to get this set up for your workspace. You'll need to:
1. Get the script from this Github gist: [builder.sh](https://gist.github.com/thomasweng15/db12693f957ecafb6eed3bb011db37a3#file-builder-sh)
2. Configure it to watch your workspace directories
3. Run it in a terminal using `bash builder.sh`
4. Start coding and enjoying ~build-on-save~

# But wait, there's more: `roslaunch` auto-restart!

After your build completes, you'll probably need to run or restart your ROS nodes to test your changes. That's another manual step we can automate.

This time, a script called `launcher.sh` runs your project's `roslaunch` command and listens periodically to make sure your ROS nodes are alive. As you make changes and get a successful build, `builder.sh` – the original script – sends a signal to kill your ROS nodes.[^2] When the ROS nodes die, `launcher.sh` will automatically restart them, grabbing your newest build. Here's an example of what restarting looks like:

```
$ bash launcher.sh
Launching roslaunch
... logging to /home/tweng/.ros/log/3674ab20-73be-11e7-b57f-b8ca3ab4b589/roslaunch-silverarm-15112.log
Checking log directory for disk usage. This may take awhile.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://localhost:39545/

...

/process_cloud_main shutdownCallback:163: Shutdown request received.
/process_cloud_main shutdownCallback:164: Reason given for shutdown: [user request]
================================================================================REQUIRED process [process_cloud_main-1] has died!
process has finished cleanly
log file: /home/tweng/.ros/log/3674ab20-73be-11e7-b57f-b8ca3ab4b589/process_cloud_main-1*.log
Initiating shutdown!
================================================================================
[publish_saved_cloud-3] killing on exit
[person_broadcaster-2] killing on exit
[process_cloud_main-1] killing on exit
shutting down processing monitor...
... shutting down processing monitor complete
done
```

`launcher.sh` notices that the node has gone down and triggers a restart:

```
Launching roslaunch
... logging to /home/tweng/.ros/log/3674ab20-73be-11e7-b57f-b8ca3ab4b589/roslaunch-silverarm-30141.log
Checking log directory for disk usage. This may take awhile.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://localhost:34414/
```

You can get `launcher.sh` [here](https://gist.github.com/thomasweng15/db12693f957ecafb6eed3bb011db37a3#file-launcher-sh). You'd run it in a terminal (`bash launcher.sh`), just like the first one.

If your builds take a lot of processing power and/or take a long time, you may need to make adjustments to this script. I personally haven't had any problems rebuilding on every save. One option if you do have problems is to rebuild only the package you are working on, and not the whole workspace.

Hope this helps and you find it useful!

---
Footnotes

[^1]: Originally inspired by Brett Victor's talk, "Inventing on Principle." Check it out a recording of it here: [video link](https://vimeo.com/36579366).

[^2]: I set the main node with the `required=true` attribute in my launch file so I only need to kill that node to stop the others.
