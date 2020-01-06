---
layout: post
title: ArduSimple RTK2B+heading - first impressions
description: "I've started looking into setting up RTK with ROS. This post describes my first steps."
modified: 2020-01-06
comments: true
tags: [ROS, Robotics]
image:
  feature: /ublox/ardusimple_base_rover.jpg
---

[ArduSimple](https://www.ardusimple.com/) provided me with their [RTK2B+heading module](https://www.ardusimple.com/product/simplertk2b-heading-basic-starter-kit-ip67/) recently. In this blog post I'm describing my experience running the first tests with these modules and my planned further tests. My end goal is to integrate the modules with ROS and get a precise positioning in 2D with extra heading information.

<!-- more -->

## RTK basics

They say that a picture is worth a thousand words, so here is a 5 minute YouTube video that should give you basic information on RTK in case you haven't heard of it before:

<iframe width="560" height="315" src="https://www.youtube.com/embed/R0Hry5kR1jY" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

What is special about the module that I'm testing from ArduSimple is the rover board that comes with 2 GPS chips, allowing to plug in two GPS antennas and figuring out the rover heading based on available information.

## Setup

The setup I've received from ArduSimple consists of the following:

* [simpleRTK2B+heading](https://www.ardusimple.com/product/simplertk2b-heading-basic-starter-kit-ip67/) - rover module
* [simpleRTK2B](https://www.ardusimple.com/product/simplertk2b-basic-starter-kit-ip65/) - base station module
* Long range radio module x2
* 3x [u-blox ANN-MB-00](https://www.ardusimple.com/product/ann-mb-00-ip67/) GNSS multiband antenna

<figure class="center">
    <img src="/images/ublox/ardusimple_base_rover.jpg" alt="ArduSimple base and rover modules">
    <figcaption>ArduSimple base module (left) and rover module (right)</figcaption>
</figure>

The ArduSimple boards are based on [u-blox ZED-F9P](https://www.u-blox.com/en/product/zed-f9p-module), a professional grade multiband GNSS modules.

The long range radio modules that I've received are XBee SX 868 - an 863-870 MHz radio modules boasting a line of sight range of 14km.

<figure class="center">
    <img src="/images/ublox/u-blox_antenna.jpg" alt="u-blox antenna">
    <figcaption>u-blox ANN-MB-00 antenna</figcaption>
</figure>

The u-blox antennas are IP67 rated have two mounting holes and an embedded magnet that makes it convenient to attach to metal surfaces.

## Testing

One advice from my drone days that stuck with me is to always plug in all antennas before powering the system. That's what I did when working with this system and I recommend that you do that too.

<figure class="center">
    <img src="/images/ublox/u_center.png" alt="u-center software">
    <figcaption>running u-center software</figcaption>
</figure>

At this point I only run some static tests. The first test I performed using [u-center software](https://www.u-blox.com/en/product/u-center), even when I put the antennas outside of my window in an urban space (conditions far from ideal due to [multipath](https://www.e-education.psu.edu/geog862/node/1721)) the software was reporting 0.23m accuracy in 2D - not bad at all!

<figure class="center">
    <img src="/images/ublox/google_earth.png" alt="GPS position being shown in Google Earth">
    <figcaption>Streaming GPS information to Google Earth</figcaption>
</figure>

Thanks to the advice from the ArduSimple team I was able to display the position on Google Earth (to do that select Menu Bar > File > Database Export > Google Earth server in u-center).

As another experiment I tested the receivers with [ROS-Agriculture ublox_f9p node](https://github.com/ros-agriculture/ublox_f9p). Amazingly it worked out of the box streaming out the position data.

After I've performed the tests I've found out about [ublox package](https://github.com/KumarRobotics/ublox) that seems to implement RELPOSNED message and hence should allow me to stream heading from my rover setup.

## Summary and next steps

I found the ArduSimple units to be a great piece of kit. I still have quite a bit of work to do to implement them on a robotic platform, here is a rough plan how I'm going to pull this off:

1. Build cases and a base station setup (ideally so that it can be placed on a tripod)
2. Test out the [ublox package](https://github.com/KumarRobotics/ublox)
4. If the package from previous point is not satisfactory: fork ROS-Agriculture ublox_f9p repository and make sure UBX-RELPOSNED message is handled and the driver publishes heading
5. Capture data by walking around with the module in hand
6. Integrate the module on my mobile robot development platform and tune the EKF to work with the RTK module
7. Enable corrections via [NTRIP](https://www.agsgis.com/What-is-NTRIP_b_42.html)
