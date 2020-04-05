---
layout: post
title: ArduSimple RTK - ROS integration
description: "In this blog post I'm describing my hardware preparations to testing the ArduSimple RTK modules."
modified: 2020-04-05
comments: true
tags: [Robotics, ROS]
image:
  feature: /ublox/boxes_4.jpg
---

As you might have seen from two of my previous posts ([1](https://msadowski.github.io/ardusimple-rtk-first-impressions/), [2](https://msadowski.github.io/ardusimple-rtk-moving-forward/)) I have been doing some testing with ublox F9P based ArduSimple RTK setup. In this blog post I'm describing how I integrated these modules with Robot Operating System (ROS).

<!-- more -->

## Introduction

Please see my [first post](https://msadowski.github.io/ardusimple-rtk-first-impressions/) in this series to learn more about my [ArduSimple setup](https://www.ardusimple.com/) the short story is I use simpleRTK2B for my base station and simpleRTK2B+heading for the rover.

<figure class="center">
    <img src="/images/ublox/ardusimple_base_rover.jpg" alt="ArduSimple base and rover">
    <figcaption>ArduSimple RTK base (left) and rover (right)</figcaption>
</figure>

## Configuration

Since I managed to break the whole configuration by overwriting it with the ublox ROS node by using a wrong config, I had to fire up the u-center application. From [ArduSimple Github Repo](https://github.com/ardusimple/simpleRTK2B/tree/master/Configuration_Files) I’ve downloaded the following configuration files:
* srtk2b_base_FW_HPG112.txt - base module
* srtk2b+heading_lite_movingbase_FW_HPG112.txt - the heading lite module (the small board), flashed by connecting to the XBEE USB
* srtk2b+heading_rover_F9P_FW_HPG112.txt - the main rover module

To flash the config files I’ve followed [instructions on ArduSimple website](https://www.ardusimple.com/configuration-files/). When writing the configuration file pay attention to the firmware version and if needed [follow these steps to update your modules](https://www.ardusimple.com/zed-f9p-firmware-update-with-simplertk2b/).

When flashing the ublox configuration following the above guides the checkbox for storing the configuration files was disabled. My assumption at the time was that F9P will automatically store the settings in the memory, however it turned out I was very wrong on that and I've noticed this only after switching back and forth between Windows and Linux numerous times.

To save the loaded config you will need to save it in View->Configuration View->CFG, Select all Devices and press the Send button at the bottom of the window.

<figure class="center">
    <img src="/images/ublox/config_save.png" alt="u-blox F9P configuration window">
    <figcaption>Saving u-blox F9P configuration</figcaption>
</figure>

## U-blox ROS driver

When searching for the ROS driver the most important feature I was looking for was the support of UBX-NAV-RELPOSNED messages that would provide the relative heading between the two rover antennas.

My ROS package of choice that would support F9P became [ublox](http://wiki.ros.org/ublox) by [Vijay Kumar Lab](https://www.kumarrobotics.org/). As I noted in the previous section using this package I've overwritten the modules configuration by running some of the example launch files in the repository. It took me a while to realise the misconfigurations in the modules but when I did I made sure to have the option `config_on_startup: false` in the .yaml file for my rover config. You will find the yaml file that I used in my experiments below:

```
debug: 1 # Range 0-4 (0 means no debug statements will print)

device: /dev/ttyACM0
frame_id: gps
config_on_startup: false
uart1:
  baudrate: 115200
rate: 1
nav_rate: 1
publish:
  all: true
```

The ArduSimple launch file I used looked as follows:

```
<launch>
  <arg name="node_name" default="gps"/>
  <arg name="param_path" default="$(find ardusimple_rover)/config/ardusimple.yaml" />
  <arg name="output" default="screen" />
  <arg name="respawn" default="true" />
  <arg name="respawn_delay" default="30" />
  <arg name="clear_params" default="true" />

  <node pkg="ublox_gps" type="ublox_gps" name="$(arg node_name)"
        output="$(arg output)"
        clear_params="$(arg clear_params)"
        respawn="$(arg respawn)"
        respawn_delay="$(arg respawn_delay)">
    <rosparam command="load" file="$(arg param_path)" />
  </node>
</launch>
```

After launching the above file I could echo `/gps/fix` topic for the GPS messages. When you do that in the RTK setup pay attention to the covariance fields, if they are very low then you most likely have a great view of the sky and the base unit is providing good corrections. To see the relative heading between the two modules you can check the `/gps/navheading` topic.

## Robot Localization

In this blog post I wanted to provide the simplest example that you could run just with the RTK hardware and no other sensors. As a test I've created a basic setup for robot_localization and navsat_transform_node to create a working TF structure. The only data I was fusing in the EKF was the GPS fix message and the relative heading between the modules.

Because I only have information from GPS this setup does not adhere to [REP-105](https://www.ros.org/reps/rep-0105.html) - we don't have any source of odometry that would be continuous. That's why you usually use GPS as one of the inputs providing a map->odom transform (more on this in the last section).

There are two nodes that I used here:
* navsat_transform_node - to produce an odometry message from gps fix
* ekf_localization_node - to create a transform between the odom and gps frames

You will find all the configuration files that I've created in [ardusimple_rover repository](https://github.com/msadowski/ardusimple_rover). There are two things to note here: because the observed rotation between the modules (`/gps/navheading` topic) is relative the navsat_transform_node will have an error in orientation. Normally you would use an IMU instead which should produce the correct results. The only two topics that are fused in the ekf are `/gps/fix` and `/gps/navheading` - in a real setup you will want to use information from your other sensors.

## Results

Due to the lockdown I was only able to perform a test in a residential area, driving a car with two rover antennas attached to the roof. I've placed the base antenna in a fixed position and let it run for a couple minutes. After manually adjusting the `yaw_offset` parameter of navsat_transform_node to align odometry with true gps coordinates I was able to get the following result in [mapviz](http://wiki.ros.org/mapviz):

<figure class="center">
    <img src="/images/ublox/mapviz.gif" alt="mapviz gif">
    <figcaption>Mapviz + U-blox ArduSimple in action (4x)</figcaption>
</figure>

Above you can see the following information:
* blue dots - raw GPS information
* green line - filtered odometry
* yellow points - location of gps frame in odom frame

In the graph below you can see horizontal gps position covariance (top) and the orientation covariance from F9P. As the car moved from an area with a wide sky view to a more urban area you can see an increase in covariance values.

<figure class="center">
    <img src="/images/ublox/covariance_graphs.png" alt="mapviz gif">
    <figcaption>Covariance values for the whole test run, courtesy of PlotJuggler</figcaption>
</figure>

## Next steps

The setup I described together with the [demo repository](https://github.com/msadowski/ardusimple_rover) should get you started on integrating and testing ArduSimple RTK boards (probably anything based on U-blox F9P too). If you wanted to integrate this solution on a mobile robot then you would run two instances of robot_localization:
* ekf map->odom - fusing information from all of your sources, including RTK + heading from RTK
* ekf odom->base_link - fusing only continuous sources of odometry (wheel odometry, IMU)

For more information on how to pull this off you can see the [robot_localization wiki](http://docs.ros.org/melodic/api/robot_localization/html/integrating_gps.html).
