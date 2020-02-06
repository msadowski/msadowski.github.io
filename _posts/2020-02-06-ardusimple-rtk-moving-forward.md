---
layout: post
title: ArduSimple RTK - preparing for testing
description: "In this blog post I'm describing my hardware preparations to testing the ArduSimple RTK modules."
modified: 2020-02-06
comments: true
tags: [Robotics]
image:
  feature: /ublox/base_plate.jpg
---

In my [previous post](https://msadowski.github.io/ardusimple-rtk-first-impressions/) I was sharing my first impressions from using [ArduSimple RTK GPS](https://www.ardusimple.com/). It's time to share some information about the small but steady progress I had made since!

<!-- more -->

## Introduction

In the last post I was testing the RTK in a very static manner by putting the GPS antennas next to the wall of a building. I'm not a fan of sticking electronic boards straight onto any rough surfaces that are unprotected in any way. My first thought on securing the electronics bits while was to 3D print [this case](https://www.thingiverse.com/thing:3551604). The issue with this case is that it's not high enough to fit the rover module that has an extra GPS board that sits below the XBee radio.

<figure class="center">
    <img src="/images/ublox/ardusimple_base_rover.jpg" alt="ArduSimple base and rover">
    <figcaption>ArduSimple RTK base (left) and rover (right)</figcaption>
</figure>

My next thought was that I could perhaps design my own case but instead I decided to design a base plate that can hold the module. If it's of any use for you and would like to 3D print it then you can grab it [here](https://github.com/msadowski/msadowski.github.io/blob/master/files/ardusimple_plate.stl). You will just need to secure the modules to the plate using 3 M3x8 screws. I recommend choosing screws with as small head as possible - the screw that goes right below XBee module is very close to the pin headers.

<figure class="center">
    <img src="/images/ublox/base_plate.jpg" alt="3D printed base plate for ArduSimple RTK modules">
    <figcaption>3D printed base plate for ArduSimple RTK modules</figcaption>
</figure>

Unfortunately the base plate won't help with any environmental protection therefore behold...

## Project Boxes

<figure class="center">
    <img src="/images/ublox/boxes_0.jpg" alt="Shiny new project boxes">
    <figcaption>Shiny new project boxes</figcaption>
</figure>

I figured that purchasing two project boxes will be way faster than designing the case from scratch and I figured I will drill two holes on every side of the box to run the necessary wires through and will be done in no time. Also the these project boxes are (were) IP66 rated.

<figure class="center">
    <img src="/images/ublox/boxes_1.jpg" alt="Project boxes with holes">
    <figcaption>Almost done here (don't think they are IP66 anymore though)</figcaption>
</figure>

After cutting the holes in the boxes it was time to place the RTK modules inside:

<figure class="center">
    <img src="/images/ublox/boxes_2.jpg" alt="Projext boxes with RTK modules inside">
</figure>

Do you see any issues here? Me too! Most of the cables are not as flexible as I had hoped and there is simply no way to acces the USB ports on the left side of the modules (the ones I was actually planning to use).

<figure class="center">
    <img src="/images/ublox/boxes_3.jpg" alt="Project boxes with RTK modules and antennas attached">
</figure>

Here are the boxes with antennas attached. You can clearly see that there is no way to get to USB ports in this configuration. Well... sometimes you've got to do what you've got to do...

<figure class="center">
    <img src="/images/ublox/boxes_4.jpg" alt="Completely butchered project boxes">
    <figcaption>From now on you can call me project boxe butcher</figcaption>
</figure>

At this point I'm pretty sure design and printing my own case would be a faster option (mostly due to the lack of proper tools). The good news is this should do the job. If it starts raining in the field the covered box should give me enough time to grab to extract it from the field without destroying the hardware.

## Lessons learned

Doing a bit of manual hardware work was a good break from all the software I've been doing in the past couple of months. Here are some things that I've learned and hopefully they are useful for you too:

* Designign a base plate before I had the boxes - in hindsight I should've waited for the boxes to arrive before desiging a base plate. That way I would have more control over the position of the modules within the box and could cut way more precise (and smaller) openings in the boxes
* I should've ordered a box with a transparent lid that way I could see the RTK status lights without needing to open up the box
* When manipulating the XBee antenna sometimes the XBee lifts up, ideally antenna should have any 'wiggle room'

## Next steps

I'm happy enough with the state of my project boxes to start running some experiments. Since the ArduSimple RTK modules are based on Ublox ZED-F9P I'm going to start with ROS [ublox package](https://github.com/KumarRobotics/ublox). I just need to set up project box for my Raspberry Pi or... not this time!

<figure class="center">
    <img src="/images/ublox/pi.jpg" alt="Raspberry Pi in a case">
</figure>
