---
layout: post
title: Hands on with slam_toolbox
description: "This blog post describes my experience running slam_toolbox on a ROS enabled mobile robot platform"
modified: 2019-09-18
comments: true
tags: [ROS, Robotics]
image:
  feature: slam_header.png
---

In the past couple of weeks, as part of a project with [Robosynthesis](https://www.robosynthesis.com/), I've been exploring [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) by [Steven Macenski](https://github.com/SteveMacenski). This post summarizes my experience so far.

<!-- more -->

Here is the description of the package taken from the project repository:

> Slam Toolbox is a set of tools and capabilities for 2D SLAM built by Steve Macenski while at Simbe Robotics and in his free time.
This project contains the ability to do most everything any other available SLAM library, both free and paid, and more. This includes:
* Ordinary point-and-shoot 2D SLAM mobile robotics folks expect (start, map, save pgm file) with some nice built in utilities like saving maps
* Continuing to refine, remap, or continue mapping a saved (serialized) pose-graph at any time
* life-long mapping: load a saved pose-graph continue mapping in a space while also removing extraneous information from newly added scans
* an optimization-based localization mode built on the pose-graph. Optionally run localization mode without a prior map for "lidar odometry" mode with local loop closures
* synchronous and asynchronous modes of mapping
* kinematic map merging (with an elastic graph manipulation merging technique in the works)
* plugin-based optimization solvers with a new optimized Google Ceres based plugin
* RVIZ plugin for interating with the tools
* graph manipulation tools in RVIZ to manipulate nodes and connections during mapping
* Map serialization and lossless data storage
* ... more but those are the highlights

## Our test setup
Here are the highlights of our setup:
* Robosynthesis differential-drive mobile robot development platform
* RPLidar A1 (hobby grade, scanning at ~7Hz)
* Onboard computer running ROS Melodic and slam_toolbox (commit: 9b4fa1cc83c2f)

<figure class="center">
    <img src="{{site.url}}/images/dev_platform.jpg" alt="Mobile robot">
    <figcaption>Robosynthesis development platform used for slam_toolbox evaluation</figcaption>
</figure>

## First impression

The first test we've run was with the default parameters that come with slam_toolbox with minor changes (frame names, and max laser range).

In ROS, as a good practice, we usually have a TF tree setup in the following way (at least as a minimum when doing SLAM):

**map -> odom -> base_link**

If you would like to know more about the transforms then [REP-105](https://www.ros.org/reps/rep-0105.html) is your friend. Sometimes we run a competition in the office who can recite it faster!

Our expectation of slam_toolbox is to provide us with a **map -> odom** transform. In our tests we'll use **odom -> base_link** transform from wheel odometry. Is it going to drift? Yes, it will! But slam_toolbox will have our back!

Here is a short gif showing our first test, driving the robot at a reasonable speed (at least for an indoor robot) around an office:

<figure class="center">
    <img src="{{site.url}}/images/slam_toolbox_odom.gif" alt="slam_toolbox mapping">
    <figcaption>Performing SLAM with slam_toolbox (replay with 3x rate)</figcaption>
</figure>

And if you'd like to see some of the raw data used during the above session then you can download the bag file [here](https://drive.google.com/file/d/1S6ceDqPf1Z_5Pq49td9y3ZL9Jpdyc1Ok/view?usp=sharing).

So far the best thing for us about this package is the performance. With the above test it was consuming less than 7% CPU on our small i7 PC that we use for robot development.

## Taking it further

What you've seen in this blog post are only the first trials with this SLAM package. If you take a look at the [configuration files](https://github.com/SteveMacenski/slam_toolbox/blob/melodic-devel/config/mapper_params_online_async.yaml) you will see that there are lots of parameters that can be tuned.

A very helpful tool that comes with slam_toolbox is the RViz plugin (Panels->Add New Panel->slam_toolbox->SlamToolboxPlugin)

<figure class="center">
    <img src="{{site.url}}/images/slam_toolbox_rviz.png" alt="Rviz plugin">
    <figcaption>slam_toolbox RViz plugin window</figcaption>
</figure>

This blog post is only the beginning of our adventure with slam_toolbox but we've liked it so much that we decided to share the results with you. Stay tuned for more information about the hardware and open source that we use!
