---
layout: post
title: Giving LaMa a shot
description: "This post describes some of my tests of iris_lama ROS SLAM package and first impressions. I also use this opportunity to compare it to slam_toolbox when possible"
modified: 2019-11-29
comments: true
tags: [ROS, Robotics]
image:
  feature: /ros_slam/lama_map.png
---

If I knew how to draw I would create a header for this blog post that would show a lama getting vaccinated. Since you won't be seeing it any time soon, let's jump into [iris_LaMa](https://github.com/iris-ua/iris_lama), a localization and mapping library that works with ROS.

<!-- more -->

## Setup

<figure class="center">
    <img src="{{site.url}}/images/dev_platform_2.jpg" alt="Mobile robot">
    <figcaption>Robosynthesis development platform</figcaption>
</figure>

Similarly to my previous experiments ([T265 test](https://msadowski.github.io/Realsense-T265-First-Impressions/) and [slam_toolbox evaluation](https://msadowski.github.io/hands-on-with-slam_toolbox/)) I've run the test described in this post on a [Robosynthesis](https://www.robosynthesis.com/) development platform.

The only difference from the previous posts is that I added an IMU, therefore the fused robot odometry should be a tad bit better. Highlights of the setup are as follows:

* Robosynthesis differential-drive mobile robot development platform
* RPLidar A1 (hobby grade, scanning at ~7Hz)
* Onboard computer running ROS Melodic and [iris_lama](https://github.com/iris-ua/iris_lama) (commit: 07808d87) and [iris_lama_ros](https://github.com/iris-ua/iris_lama_ros) (commit: 54df5359)
* Pixhawk autopilot used as an IMU

## First LaMa impressions

LaMa is the library developed by Intelligent Robotics and Systems (IRIS) Laboratory at University of Aveiro. The package is maintained by [Eurico F. Pedrosa](https://github.com/eupedrosa) and is open sourced under BSD 3-Clause licence.

Getting started with LaMa was very simple - pull the two repositories, build them, change couple of parameters (scan_topic, global_frame_id, odom_frame_id, base_frame_id) to match my platform and run it.

There are 3 nodes of interested in the ROS package:
* slam2d_ros - online SLAM
* loc2d_ros - localization in the known map
* pf_slam2d_ros - particle filter slam

In this blog post I'll focus on the first two nodes.

### Online SLAM

<figure class="center">
    <img src="/images/ros_slam/lama_map.png" alt="Map created with LaMa">
    <figcaption>SLAM with LaMa</figcaption>
</figure>

The first impression I got using this SLAM package was that it "just works". Out of the box without tuning any parameter values except of the frame name changes the SLAM seemed to perform very well. For the full parameter list you can see the [project readme](https://github.com/iris-ua/iris_lama_ros/blob/master/README.md). I found that the number of parameters you can tune is smaller than in case of other packages I used so far (looking at you Cartographer). The good thing about it is that you don't need to spend so much time tuning the parameters to get good results, on the other hand you might not have so much flexibility.

### Localization

With the localization node you can start a map_server and the localization node will work with the map coming from the server. Currently the node doesn't support global localization (as per Readme.md) but instead you can send the robot pose on the `/initialpose` topic (this means that if you set the pose estimate from RVIZ it will work too). I found that even if you set the pose ~1-2m from the real robot pose it converged to the true pose as the robot is moving around.

## Apples to oranges

Since I already covered slam_toolbox in one of my [previous posts](https://msadowski.github.io/hands-on-with-slam_toolbox/) I've decided I would try to see how they compare. Do you think arbitrary comparisons of two SLAM results is meaningless? Me too! But I'm going to do that anyway.

<iframe width="560" height="315" src="https://www.youtube.com/embed/Cgcl3LcFnEs" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

Above you will find a video comparison of online SLAM with iris_lama compared to slam_toolbox. I run the two solutions with default configs and only changed the platform related settings (mostly the names of frames and LiDAR range). Here are some of my observations on LaMa:
* Visually the position estimate of LaMa looks a bit smoother (less discrete jumps)
* Erroneus readings from my LiDAR are added to the map in LaMa (you can see points appearing way outside of the map)
* The very small features seem to be missing from te map (you can see that well in the upper right corner of the map)

It's quite possible that some of the things above can be tuned through parameters, however I didn't find any obvious parameters described in the readme file.

Another thing I wanted to know is how does the position estimates compare. For this reason I have used the [evo Python Package](https://michaelgrupp.github.io/evo/) and described my workflow in full in my [previous post](https://msadowski.github.io/Comparing-SLAM-with-ROS-evo/). In the screenshots below I have used the output from iris_lama as a ground truth:

<figure class="half">
	<img src="/images/ros_slam/rpe_3.png">
	<img src="/images/ros_slam/rpe_4.png">
	<figcaption>Relative Pose Error results on my data</figcaption>
</figure>

What these images tell us is that most of the time the output of the both packages is quite close to one another (you can right click on the image to see it in full size). The issues with this comparison is that without ground truth we can't say anything more about it, which brings me to the next section.

### CPU usage

I think the biggest advantage of iris_lama is its low CPU usage (In the project readme author says that it runs great on Raspberry Pi 3B+). Here is the comparison of CPU usage of iris_lama (left) and slam_toolbox (right).

<figure class="half">
	<img src="/images/ros_slam/lama_cpu.png">
	<img src="/images/ros_slam/slam_toolbox_cpu.png">
	<figcaption>CPU usage comparison between the two packages</figcaption>
</figure>

You will see that the LaMa CPU usage peaks at around 15% and slam_toolbox at 450%. I hope that one day I will get to repeat this experiment in a larger environment and see how both packages manage it. I grabbed the CPU usage of both packages using [cpu_monitor](https://github.com/pumaking/cpu_monitor) package and running it along the bag file with raw data and each SLAM package. Then I ploted the output using [PlotJuggler](https://github.com/facontidavide/PlotJuggler).

## Links

Here are some links that can provide you with further reading or allow you to replicate my results.

* [ROS discourse discussion](https://discourse.ros.org/t/announcing-lama-an-alternative-localization-and-mapping-package/10916) on LaMa
* [A bagfile with raw data I used for this blog post](https://drive.google.com/file/d/1GLs5PdKEzpkgN3aeGtPtEOW46JrIaXBr/view?usp=sharing)

## Next steps

I've recently received a [simpleRTK2B+heading](https://www.ardusimple.com/product/simplertk2b-heading-basic-starter-kit-ip67/) module from [ArduSimple](https://www.ardusimple.com/) which should get me sub centimeter level position accuracy. This should be a decent source of ground truth for my follow up experiments. Stay tuned for more information in the next post!
