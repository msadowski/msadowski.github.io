---
layout: post
title: YDLIDAR X4 - ROS review
description: "Lately I had a chance to thoroughly test YDLIDAR X4 lidar with the relevant ROS packages. This post summarizes my experience working with this cheap LIDAR and highlight its strengths and weaknesses."
modified: 2019-01-09
comments: true
tags: [ROS, Robotics, Sensors]
image:
  feature: ydlidar/ydlidar.jpg
---

Recently I gave YDLIDAR X4 a spin, in this post I'm documenting my experience using it together with [EAIBOT ydlidar ROS package](https://github.com/EAIBOT/ydlidar).

<!-- more -->

## Hardware specification

|Parameter       |  Value     |
|:---------|:---------|
|Scanning range | 360° |
|Range | 0.12-10m |
|Range resolution | < 1% of distance |
|Average resolution | ~0.5° |
|Scan frequency | 6-12Hz|
|Range frequency | 5000Hz |
|Max current | 480mA |
|Laser wavelength | 775-795nm|
|Laser power | 3-5mW|
|Price | ~100 USD |

The hardware specification makes YDLIDAR probably the best value for the price for a single plane 2D LiDAR.


## ROS package

I tested the LIDAR with [EAIBOT ydlidar package](https://github.com/EAIBOT/ydlidar)(ecfd95e2). What I like about the package is that it should automatically work with all the YDLIDAR scanners (in the code the driver polls device information from the LIDAR and uses it to fill in some of the settings). Another thing I liked a lot is the amount of parameters exposed in the launch file, allowing to customize some of the settings:

{% highlight xml %}
  <node name="ydlidar_node"  pkg="ydlidar"  type="ydlidar_node" output="screen">
    <param name="port"         type="string" value="/dev/ydlidar"/>
    <param name="baudrate"     type="int"    value="115200"/>
    <param name="frame_id"     type="string" value="laser_frame"/>
    <param name="angle_fixed"  type="bool"   value="true"/>
    <param name="low_exposure"  type="bool"   value="false"/>
    <param name="heartbeat"    type="bool"   value="false"/>
    <param name="resolution_fixed"    type="bool"   value="true"/>
    <param name="angle_min"    type="double" value="-180" />
    <param name="angle_max"    type="double" value="180" />
    <param name="range_min"    type="double" value="0.08" />
    <param name="range_max"    type="double" value="16.0" />
    <param name="ignore_array" type="string" value="" />
    <param name="samp_rate"    type="int"    value="9"/>
    <param name="frequency"    type="double" value="7"/>
  </node>
{% endhighlight %}

The parameters I appreciated the most are the frame_id and angle_min and max that allow to specify the range of measurements you are interested in (useful in cases where part of the robot obstructs the scanner).

There are also issues with the package. First of all changing the frequency parameter doesn't change the behaviour ([relevant GitHub issue](https://github.com/EAIBOT/ydlidar/issues/8)). The code itself could also use some refactoring to remove some of the magic numbers and breaking the code a bit more into functions. A nice to have would be an addition of Dynamic Reconfigure parameters so that the scanning parameters could be changed at run time.

## General feel

Given the price there is nothing to complain about when it comes to X4. I've been pretty happy with it so far, if you are looking for a low cost 2D LIDAR and you don't mind a relatively slow update rate then I would say the X4 is worth a shot.

Looking just through the [YDLIDAR website](http://ydlidar.com/product/X4) you might not realise that the usb port is not directly connected to the LIDAR but instead an adapter board is used:

<figure class="half center">
	<img src="{{site.url}}/images/ydlidar/x4_usb.png" alt="X4 USB connection">
	<img src="{{site.url}}/images/ydlidar/x4_adapter.png" alt="X4 micro USB adapter">
	<figcaption>What you'd think you get vs. what you actually get</figcaption>
</figure>

When it comes to usability I noticed that if the LIDAR is initialized very close to obstacles then a failure case can be triggered. The observable result is the ROS node outputting a 'scrambled' scan that has no correlation with reality (for example all measurements form a straight line around the scanner). When that happened killing the node didn't stop the LIDAR from spinning. Other than this failure case I didn't have any issues, in 99% of cases the LIDAR worked reliably.

<figure class="center">
  <img src="{{site.url}}/images/ydlidar/lidar_scan.gif" alt="YDLIDAR X4 scanning">
  <figcaption>YDLIDAR X4 scan visualization</figcaption>
</figure>

The X4 also proved to be working relatively well for SLAM (Simultaneous Localization and Mapping). As long as you don't move too fast (remember the 7Hz scan update rate) it works reliably.

<figure class="center">
  <img src="{{site.url}}/images/ydlidar/x4_slam.gif" alt="YDLIDAR used for SLAM">
  <figcaption>Performing SLAM with YDLIDAR X4</figcaption>
</figure>

## Final thoughts

I really enjoyed working with this hardware and software. It's true that there are some issues with it but for the price I doubt you will be able to find anything better. None of the issues I found would be a showstopper for an indoor application, where the robot doesn't need to move to fast. I think it's a great piece of hardware, especially for budget hobbyist projects.

Unfortunately I didn't get a chance to test the X4 outside in very sunny conditions - according to the documentation and my experience such conditions will affect the scanner's performance. If you had a chance to use it outside please let me know in the comments how did it go!