---
layout: post
title: Comparing SLAM results using ROS and evo
description: "In this blog post I look into comparing results from two ROS SLAM packages using evo Python package on data from processed bag files."
modified: 2019-11-21
comments: true
tags: [ROS, Robotics]
image:
  feature: ros_slam/rpe_2.png
---

While working on another blog post I fell into a rabbit hole of comparing SLAM packages using ROS. This blog post briefly describes how I managed to compare the results of two SLAM packages by using [evo Python package](https://michaelgrupp.github.io/evo/).

<!-- more -->

## Background

Recently I've started working a lot with various SLAM packages in ROS using the [Robosynthesis](https://www.robosynthesis.com/) dev platform. I figured that opening RVIZ, displaying the robot model in the map frame and saying "yeaah, that looks about right" might not be necessairly the best way to evaluate the quality of SLAM.

What my data lacks is groundtruth as at the moment I don't have a way to capture it. This post describes the way to evaluate output of two SLAM methods against one another.

## evo

I found evo, while doing research for my newsletter Weekly Robotics and featured it in [issue #58](https://weeklyrobotics.com/weekly-robotics-58).

As per the description on [project website](https://michaelgrupp.github.io/evo/):

> This package provides executables and a small library for handling, evaluating and comparing the trajectory output of odometry and SLAM algorithms.

The thing most interesting for us in the scope of this blog post is that it supports bag files!

## Gathering the data

To gather the data I run the robot around my office with all my mapping stack running and made sure that the robot surveyed enough area to build a good map. The next step was to run 2 SLAM packages using the data from my .bagfile. To do it correctly you need to ensure your bag contains only the data of interest, hence rosbag filter comes handy:

```
rosbag filter slam.bag bag_filtered.bag "topic == '/clock' or topic == '/rr_robot/joint_states' or  topic == '/odometry/filtered' or topic == '/scan' or (topic=='/tf' and m.transforms[0].header.frame_id != 'map' or topic=='/tf_static')"
```

The most important things we need from the bag are clock, laser scan topic and the whole tf structure except the map->odom transform if your setup follows REP-105.

After these steps we shold have a set of raw data that we can use to run some SLAM packages from the recorded input. The main reason I find it useful is that it allows tuning the parameters to some extend and rapidly visualise the change in quality.

## Rebagging the data

The next step that I performed was to run the slam package of choice with the recorded data. Couple of pointers:
* Make sure to set use_sim_time parameter to true
* Run your bagfile with --clock argument
* Visualise everything in rviz to make sure your data is working correctly. I advise displaying the map and pose of the robot

When running the SLAM node make sure that you record the bag file (let's call it run_1.bag) of your run, this time record all the data as we won't be filtering this again.

After you have grabbed the run_1 you can re-do the experiment with another package and create run_2.bag file.

## Extracting the data

Evo currently works only with geometry_msgs/PoseStamped, geometry_msgs/TransformStamped, geometry_msgs/PoseWithCovarianceStamped and nav_msgs/Odometry topics. That means that in case of most SLAM packages that I used to date you can't use it directly as most of them provide map->odom tf transform.

Luckily evo repository contains [a file](https://github.com/MichaelGrupp/evo/blob/master/contrib/record_tf_as_posestamped_bag.py) we can use for our needs. In short this script will look at the bag file and will create a pose topic between two frames of interest. This is exactly what we need now and we will run it twice on our run_*.bag files:

WARNING! The methods below are destructive, make sure you keep copies of your original bag files.

```
./record_tf_as_posestamped_bag.py  --lookup_frequency 15 --output_topic run_1_pose --bagfile run_1.bag map base_link
```

```
./record_tf_as_posestamped_bag.py  --lookup_frequency 15 --output_topic run_2_pose --bagfile run_2.bag map base_link
```

The above scripts assume that your map frame is called 'map' and the base_link is a reference point on your robot and is called 'base_link'. Running the script our bag file will only contain a single topic named run_*_pose.

## Merging bags

At the moment it is not possible to run evo with two bagfiles as input therefore we will need to merge them. The way I did it was by running [this script](https://gist.github.com/troygibb/21fec0c748227eec89338054e6dd1833):

```
./bagmerge.py run_1.bag run_2.bag -o merged.bag
```

## Working with evo

Having the data with two topics (run_1_pose and run_2_pose) we can now get some metrics on our solutions using commands like:
* 'evo_ape' - absolute pose error
* 'evo_rpe' - relateive pose error

You would run them as follows: `evo_rpe bag merged.bag run_1_pose run_2_pose --plot` and you would see the following output:

<figure class="half">
	<img src="/images/ros_slam/rpe_1.png">
	<img src="/images/ros_slam/rpe_2.png">
	<figcaption>Relative Pose Error results on my data</figcaption>
</figure>

What I'm showing here is probably a tip of the ice berge, for more information about evo please see [the wiki](https://github.com/MichaelGrupp/evo/wiki).

## Results discussion

The most important thing I'm lacking in my comparison is the groundtruth. Without it we can't be certain which of the results is more correct but if you had an access to a system that could provide a groundtruth this approach should be working very well and you should be able to use groundtruth straight out of the box.

I think that the process I described in this blog post is a bit complex at the moment but it should be quite easy to streamline it. If you have some ideas how to improve it let me know!
