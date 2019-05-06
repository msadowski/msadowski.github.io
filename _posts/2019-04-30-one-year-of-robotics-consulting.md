---
layout: post
title: One year of working as a Robotics consultant
description: "It has been one year since I decided to go solo and start working as a Robotics consultant/contractor. In this post I'll sum up my experience so far"
modified: 2019-04-30
comments: true
tags: [ROS, Robotics]
---

It has been one year since I decided to go solo and start working as a Robotics consultant/contractor. In this post I'll sum up my experience so far.

<!-- more -->

## Background

In March 2018 I had a realization that I was not growing professionally in the direction that I'd hoped and with a 6 months runway money and a great girlfriend willing to support me I decided to take a leap of faith and handed my 1 month notice without having any contracts lined up. Although risky everything had worked out much better than expected! I had my first contract lined up 2 weeks after handing in my notice and I've been working with that company ever since.

## Values

Just before deciding to go solo I've read ["Let My People Go Surfing"](https://msadowski.github.io/lets-learn-from-patagonia/) that inspired me to join [1% For The Planet](https://msadowski.github.io/one-percent-for-the-planet/), pledging to donate at least 1% of my income to environmental organizations.

The other values I set for myself are: constant learning/improvement and high quality of service. Given that most of my projects are long term (3+ months) I like to believe that my customers are satisfied and lately I'm feeling like I'm learning something new every day I'd say that those values have been working well so far!

One reason why I wanted to have values defined early on is that I believe it makes it easier to stick to them. In the hopeful event that I'll turn my one-man activity into a proper business the values will already be established and I'll want to carry them on.

## UpWork

I've been working as a freelancer on UpWork since I started my activity and it had worked quite well! Robotics being a niche field on UpWork there is a low number of contracts available but on the other hand there are not many applicants. On average I apply to 2 contracts a month and on average I'm invited 4 times for a job interview through the platform (most of the time it's for a 10+ hour tasks for $50 though).

UpWork charges 20% fee for the first $500 earned (after that it drops to 10% until $10k when it drops to 5%). The advantage of it is that UpWork works as an intermediary and guarantees payment four hourly contracts.

After about 6 months of working on UpWork I've earned a Top Rated badge and a 100% job success score.

## Weekly Robotics

In August 2018 I've started Weekly Robotics newsletter as a side project. I realized that I do lots of research online on Robotics related news and technologies and that I could as well write those things down and share them with like minded people.

At a time of writing this post there had been 36 standard issue and a special [Robotics in 2018](https://weeklyrobotics.com/weekly-robotics-2018) issue. Since starting we partnered with [MIT Robo-AI exchange](https://robo-ai.org/gallery) and [RobotUnion](https://robotunion.eu/robotunion-community-partners/). The website has 1k unique visits a month and 740 subscribers through mailing list.

## Near Future Plans

When starting my activity in Geneva I had to create a business plan. My original plan had 3 phases:

1. Wind up phase in which I dedicate 100% of my working time to consulting (+2 extra hours per week of technical work donated to non-profits)
2. Project focus, where I focus part of my time on my own projects (such as robot platforms, different driving mechanisms etc.)
3. Assuming any of the ideas from point 2. is likely to work out well - incorporating a company around that idea and perhaps carrying on with consulting to be sustainable

What I've learned early on is that it's impossible to do a clear cut as described above, especially because I came up with Weekly Robotics, which should fall into 2nd stage, however I still needed to focus a lot on consulting and in the end I ended up working way more than planned (Weekly Robotics had become a proper side project, always done in what I consider my spare time). Only in the past 1 month I started modestly looking into 2nd phase. I'm expecting to stay in the 2nd phase for quite some time, developing projects that will allow me to test various sensors, robot setups, algorithms and hopefully will allow me to better work with remote customers.

## Let's Talk Tech

When starting out I was describing myself as "Robotics and Drones Consultant" but so far every project but one that I've taken had been related to ROS or Pixhawk (either Px4 or ArduCopter). So far I've worked on the following projects:

* C# software for interfacing with 2 Pan-Tilt-Zoom cameras and tracking regions of interest
* Drivers for Pixhawk/ArduCopter for a time of flight sensor
* ROS based software and architecture development for mobile robots by [Robosynthesis](http://robosynthesis.com/)
* A 2D/3D handheld mapping module
* Helping [Greenzie](https://www.greenzie.co/) implement ros_control and hardware_interface for their autonomous mower kit
* Two small projects related to VTOL aircraft utilising Pixhawk autopilots

All in all close to 95% of the work I'm doing is related to ROS (Robot Operating System). Here are some things that I've learned:

* In mobile differential drive / skid steering robots a properly implemented [ros_control](http://wiki.ros.org/ros_control) will massively speed up prototyping. Once ros_control is properly implemented driving the robot around becomes a breeze and thanks to [move_base](http://wiki.ros.org/move_base) you can use existing navigation packages, allowing you to send the robot to a desired waypoint and you will get obstacle avoidance for free too.
* Velocity mux (such as [yocs_cmd_vel_mux](http://wiki.ros.org/yocs_cmd_vel_mux)) provide easy interface for velocity command arbitration. If you want the user to be able to override autonomous navigation then this could be a part of your solution.
* [PlotJuggler](http://wiki.ros.org/plotjuggler) is the best software for graphing data that I've ever seen and it saved me countless hours while debugging issues.
* [tmux](https://github.com/tmux/tmux) combined with [tmuxp](https://github.com/tmux-python/tmuxp) make for a powerful customizable shell manager allowing to easily synchronize terminals (very useful when you need to ssh to 4 computers at the same time), run 4 launch files in separate windows etc.
* Nodelet all the things! ([link](https://www.clearpathrobotics.com/assets/guides/ros/Nodelet%20Everything.html))

When it comes to predictions and my TODO list there are 3 things I will be focusing on:

* ROS2 - I'm seeing it gaining quite a lot of momentum. If you happen to develop ROS drivers or software for your products now it's the best time to look into using it
* Robot prototyping and development - the robot platform that will allow me to test some ideas, algorithms and sensors is coming. More information soon!
* Behaviour tree libraries - especially [BehaviourTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP/) is an interesting library to explore for developing complex robotics behaviours.

Other than that the plan is simple: stay focused on creating quality solutions, keep donating to environmental organizations and keep learning!
