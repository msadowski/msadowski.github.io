---
layout: post
title: "IMUs and LiDARs - Not Uncommon Pitfalls"
description: "In this blog post, we will look at some issues you might come across when working with LiDARs and IMUs in your robotics projects."
modified: 2022-02-24
comments: true
tags: [Robotics]
# image:
#   feature: M_consulting_cropped.png
---

While preparing my ROS2 course on mobile robots I was not able to find quality information about basic sensors that can be used for mobile robots and the most important considerations on using them. In this short blog post, I'll discuss some of the issues I have come across while using IMUs and LiDARs in tens of robotics projects.

<!-- more -->

## IMUs

If you are working in mobile robotics it's pretty much given that you will use an IMU on your robot. Over the years I've worked with multiple sensors such as:

* [MPU6050](https://www.sparkfun.com/products/11028)
* [Bosch BNO055](https://www.bosch-sensortec.com/products/smart-sensors/bno055/)
* [PhidgetSpatial Precision](https://www.phidgets.com/?tier=3&catid=10&pcid=8&prodid=1204)
* [Xsens MTi-630-DK](https://www.xsens.com/products/mti-600-series) - last used for my blog post on [3D Mapping](https://msadowski.github.io/3d-mapping-with-ros/)

When choosing a sensor, your number one consideration will be most likely price, if you are on the market for some sensors the IMUs above will most likely introduce you to the whole range of prices. Please don't consider the list above as advice, as everything depends on the fine details of your project. Most likely you will want to take a look at such features as:

* Update rate - for wheeled mobile robots ~50Hz is enough, for UAVs 100-500Hz should do depending on aircraft type
* Whether the attitude fusion happens onboard the sensor
* Available interfaces - USB is probably the easiest if you are thinking of using ROS
* Resolution, maximum readings, gyro drift etc.
* Calibration requirements - more on this in the next section

### IMU gotchas

In my experience so far, IMUs are **the sensors** causing the most trouble when integrating them on mobile robots. Let's go through some common pitfalls one by one:

**1. Magnetic interferences**

In my first job, we were integrating a drone autopilot with an onboard IMU on a helicopter platform. After many days of trial and error, we were not able to tune the heading controller that would make the helicopter keep a fixed heading while in the air. Only by accident, we've noticed lots of noise in the heading component in the IMU data and then when we were landing the aircraft we've noticed that, actually, these magnetometer errors have a period and it's tied to the rotation speed of the main rotor! We have not seen this behaviour in the smaller model, which lead us to the discovery that in the model we had used ferrous metal rods in the blades! That would explain the noise.

<figure class="center">
    <img src="/images/applied_sensors/RH80E10XW_F_01.jpg" alt="Image of TREX 800E helicopter">
    <figcaption>Rough dimensions of the helicopter in question (not the exact model). Source: <a href="https://www.align.com.tw/index.php/helicopter-en/trex800/">align.com.tw</a></figcaption>
</figure>

**HINT:** If you are using a magnetometer in your robot - always move it as far away as possible from sources of the magnetic field. The most common source would be moving ferrous metals, actuators (magnets!) and high-current wires.

**2. Vibration**

Your robot most likely vibrates. The amplitude of the vibrations will depend on materials used, type of terrain and the drive train (among many other factors). In drones, we had a rule that when you plot all acceleration axes, and X or Y axis measurements are touching the Z-axis measurements during hover then your vibration levels are too high.

<figure class="center">
    <img src="/images/applied_sensors/vibrations_s500_accel.8df61160.png" alt="Image of TREX 800E helicopter">
    <figcaption>Example of vibrations that are definitely too high. Source: <a href="https://docs.px4.io/master/en/log/flight_review.html">docs.px4.io</a></figcaption>
</figure>

Transferring these rules onto mobile robotics:

**HINT:** If your robot drives on a flat planar surface and you see acceleration readings overlap between X/Y and Z axes then you definitely need to dampen the vibrations.

To dampen these kinds of vibrations you could use rubber offsets or thick double-sided tape.

**3. Axes definition**

This point is highly specific to software, but many robotic systems I've been working on had issues with wrongly defined IMU axes at one point or another.

**HINT:** Always double-check the IMU axes defined in your software vs. the axes of your physical device.

**4. Calibration**

Some IMUs will require you to perform calibration (usually rotating the sensor in some way in all-axes), however, some IMUs will not store this information (looking at you [BNO055](https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/device-calibration)). Once your IMU is mounted on a target system, you might not be able to rotate it along all axes, so you need to make sure this will not cause you some issues.

<figure class="center">
<video controls="controls" class="center" style="width:100%">
    <source src="/images/applied_sensors/roll.mp4" type="video/mp4">
</video>
<figcaption>3D IMU calibration after you've fitted it on the car. Source: <a href="https://www.youtube.com/watch?v=iVRmFQixqsc">YouTube (Top Gear)</a>
</figcaption>
</figure>

## LiDARs

If you are reading this article from top to bottom (why wouldn't you?), then here starts the fun! With LiDARs we are entering a space of 'smart robotics', where our robots can start reasoning around their environments and can know where they are.

Without going into too much detail ([Wikipedia has got you covered for this](https://en.wikipedia.org/wiki/Lidar)), with LiDARs we are talking about a light pulse is emitted and we calculate the time it takes for it to come back (in some cases we might look at the phase shift of light too).

When we talk about LiDARs on the market we will find the following types:

* Single point distance sensors (e.g. [Terabee TeraRanger Evo](https://www.terabee.com/shop/lidar-tof-range-finders/teraranger-evo-15m/), [Lightware SF11/C](https://lightwarelidar.com/collections/lidar-rangefinders/products/sf11-c-100-m)) - provide single point measurements
* 2-D scanners (e.g. [RPLidar A1](https://www.slamtec.com/en/Lidar/A1), [Hokuyo UTM-30LX](https://www.hokuyo-aut.jp/search/single.php?serial=169)) - provide single plane measurements
* 3-D scanners (e.g. [Ouster](https://ouster.com/products/scanning-lidar/os0-sensor/), [Velodyne Puck](https://velodynelidar.com/products/puck-lite/)) - provide measurements in multiple planes
* Scanners with non-repeating patterns (e.g. [Livox Mid-40](https://www.livoxtech.com/mid-40-and-mid-100)) - provide measurements as a non-repeating pattern

The choice of the sensor should depend on your application, interfaces you want to support, the processing power available etc.

### LiDAR gotchas

Before I worked with LiDARs I didn't know that light can be so annoying! Here are some things you might (but hopefully won't) come across.

**1. Eye fatigue**

Now, I don't have any evidence for this, but on numerous occasions, I have experienced eye fatigue while working with LiDARs. It feels like your eyes are a bit numb, my colleague describe the feeling as 'pain, but without the pain component'. Technically, most LiDARs you will come across will be eye-safe, but I would still recommend not keeping them at your eye level. If anyone knows any research about long term issues that could result from these, please feel free to let me know.

<figure class="center">
    <img src="/images/applied_sensors/bart.png" alt="Blind Bart">
    <figcaption>Me, the first time setting up the robot for half a day and keeping the LiDAR at my eye level</figcaption>
</figure>

**2. 'Flip effect'**

This is one of the issues that you will rarely come across but if you do it will be a huge pain in the ass to rectify. I'm actually not aware of anything you can reliably do to filter this kind of data in all cases. What happens here, is that the sensor reports a way smaller distance than it should, as shown in the picture below:

<figure class="center">
    <img src="/images/applied_sensors/flip_effect.png" alt="Sensor ranges when flip effect occurs">
    <figcaption>The 'flip effect' illustrated</figcaption>
</figure>

The real distance of the sensor is red + green (expected distance), but the sensor report just the green distance. The only way to fix this problem that I know of is to update sensor firmware.

**3. Field of view shenanigans**

Your LiDAR emitter will have a certain field of view (most likely a circular one, about 1-3 degrees). Looking at the picture below, there is a sensor with a field of view, and an object that is only partially in view. What distance would you expect the sensor to provide?

<figure class="center">
    <img src="/images/applied_sensors/sensor_fov_averaging.png" alt="Sensor field of view with a wall and an object in between the wall and the sensor">
    <figcaption>Sensor and it's field of view</figcaption>
</figure>

The answer is: it depends on your sensor! Some will provide the first return (Y), some will provide both Y and X (that's how you can [map tree canopies with LiDAR](https://www.researchgate.net/figure/Example-LiDAR-point-clouds-of-a-canopy-tree-located-in-Permanent-Sample-Plot-PSP-14_fig3_341735422)) and some will average all distances within the field of view providing the result Y < Result < X.

Whether any of these will be a problem for you depends entirely on your application.

**4. Maximum distance vs. environment**

The datasheets of LiDARs will usually specify the maximum distance, but you will not necessarily see the same maximum distance in your application. Usually, the specs provided are for targets with some defined reflectivity. If you direct your sensor at a black target (low reflectivity), you probably won't be able to reach the maximum distance shown in the datasheet.

Another issue that will hamper your measurements are outside conditions. The chart below will show you which wavelengths are overly present in solar radiation.

<figure class="center">
    <img src="/images/applied_sensors/Solar_Spectrum.png" alt="Sensor field of view with a wall and an object in between the wall and the sensor">
    <figcaption>Solar radiation spectrum. Source: <a href="https://commons.wikimedia.org/wiki/File:Solar_Spectrum.png">Wikimedia Commons</a> </figcaption>
</figure>

Most likely, the sensor of your choice will fall into one of the valleys in the chart. If the manufacturer of your sensor is not providing the wavelength of the sensor. consider it to be a red flag. Combining everything we've covered in these subsections, you might learn that having a sensor operate in full sun, looking at tall grass might yield the maximum range that will be suboptimal for your application.

## Outro

This about sums it up, when it comes to some pitfalls I've run into when working with LiDARs and IMUs. Robotics being robotics there are hundreds of other things that can go wrong in your setup, hopefully with some patience and good detective work you'll be able to get to the bottom of issues you come across in your setups.

Now, let's make some robots!

<figure class="center">
    <img src="/images/applied_sensors/turtle_wr.jpeg" alt="Turtlebot wearing T-shirt 'I make robots'">
    <figcaption>Turtlebot wearing a <a href="https://shop.weeklyrobotics.com/">Weekly Robotics T-shirt</a></figcaption>
</figure>
