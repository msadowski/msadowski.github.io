---
layout: post
title: YDLIDAR X2 - ROS review and running it with Cartographer
description: "I've run some tests on YDLIDAR X2. This blog post summarizes the experience and shows the LiDAR working with Google Cartographer."
modified: 2019-05-23
comments: true
tags: [ROS, Robotics, Sensors]
image:
  feature: ydlidar/x2_0.jpg
---

This month I received a sample X2 LiDAR from YDLIDAR. In this blog post I'm documenting my experience using it and using it for SLAM using ROS and Google Cartographer.

<!-- more -->

## Hardware specification

|Parameter       |  Value     |
|:---------|:---------|
|Scanning range | 360° |
|Range | 0.1-8m |
|Range resolution | 2% of distance |
|Scan frequency | 5-8Hz|
|Range frequency | 3000Hz |
|Max current | 500mA |
|Laser wavelength | 775-795nm|
|Laser power | 3-5mW|
|Price | ~89 USD |

## General feel and comparison to YDLIDAR X4

X2, similarly to X4, is a 360° LiDAR that is belt driven by a small motor. Structurally the LiDAR is very similar to the X4 that I [reviewed back in January](https://msadowski.github.io/ydlidar-x4-review/), however the footprint is a bit smaller as you can see in the photo below.

<figure class="center">
	<img src="{{site.url}}/images/ydlidar/x2_1.jpg" alt="Comparison of X4 and X2 LiDARs">
	<figcaption>YDLIDAR X4 (left) vs YDLIDAR X2 (right)</figcaption>
</figure>

Compared to X4 the X2 has a lower scanning range (8m compared to 10m) and slightly higher resolution (2% for X2 compared to 1% for X4). The default scan rate of X2 7Hz, however it can be adjusted using PWM signal.

What I especially like about the X2 is the upgraded interface box that is now enclosed in plastic and the data port is now a USB C port (there is still an extra micro USB port for power, which I didn't need to use so far).

## ROS package

When testing the LiDAR I was using the official [ydlidar package](https://github.com/YDLIDAR/ydlidar/tree/s2) (for early adopters make sure you are on s2 branch for X2).

The package I tested with is the same as I used in my [review of X4](https://msadowski.github.io/ydlidar-x4-review/) so instead of focusing on it again let's jump to SLAM setup and making it work with Cartographer.

## YDLIDAR X2 Cartographer setup

[Cartographer](https://google-cartographer.readthedocs.io/en/latest/) "is a system that provides real-time simultaneous localization and mapping (SLAM) in 2D and 3D across multiple platforms and sensor configurations". I found Cartographer to be a bit more resilient when working with LiDARs that have a relatively slow update rate. In the video below you can see how did cartographer work with X2 with the setup that I used:

{::nomarkdown}
<iframe width="560" height="315" src="//www.youtube.com/embed/pa7j01aq9po" frameborder="0" allowfullscreen></iframe>
{:/nomarkdown}

Close to 10th second of the video you can see at the top that by mistake I pointed the LiDAR at the ceiling, however after tilting it back and keeping it steady Cartographer corrected it and localized properly.

It's quite likely that the setup can be tuned to work even better. If you would like to start where I left off here is the Cartographer lua script that I used:
{% highlight lua %}
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_footprint",
  published_frame = "base_footprint",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 10,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 7
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 10
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

return options
{% endhighlight %}

You can view the full project source code on [GitHub](https://github.com/msadowski/x2_cartographer). If you would like to check the data yourself I recorded a bag file of a run you saw in the video above. It is located in a bags file in the root of the repository.

## Final thoughts and further tests

X2 is a perfect LiDAR for the price, especially for hobby or classroom use. I expect that this LiDAR should work perfectly for slowly moving (< 1 m/s) robots as an only source of odometry information. For anything requiring to move faster quite likely additional sources of odometry will be required.

In the future I would love to see an update to the ros package that would add a dynamic reconfigure support and maybe even add support for nodelets.

Unfortunately I didn't get to test the LiDAR in full sunlight (we didn't get much of it since I started testing the unit). Sometimes I saw some noise in the scans in the direction of windows but unfortunately I don't have any hard data at this stage. As soon as I get a chance to tet the LiDAR in harsh outside environment I'll update this blog post.

## Useful links

* YDLIDAR ROS repository: [https://github.com/YDLIDAR/ydlidar/](https://github.com/YDLIDAR/ydlidar/)
* YDLIDAR webpage: [http://ydlidar.com/](http://ydlidar.com/)
* YDLIDAR X2 on Amazon: [https://www.amazon.com/dp/B07S35L2NG](https://www.amazon.com/dp/B07S35L2NG)
* Cartographer ROS documentation: [https://google-cartographer-ros.readthedocs.io/en/latest/](https://google-cartographer-ros.readthedocs.io/en/latest/)
