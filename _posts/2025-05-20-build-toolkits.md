---
layout: post
title: "Build Toolkits"
description: "In this article I'll try to convince you to move your developers away from provisioning new devices, and instead create power-user interfaces for your organization."
modified: 2025-05-20
comments: true
tags: [Robotics]
image:
  feature: toolkit/power_user_interface.png
---

Build Toolkits

Recently, I’ve been thinking about some good practices in robotics software development and realized that one of the most impactful is the one I learned early on. The lesson is: **Build Toolkit(s)**.

<!-- more -->

## Background

In 2014, I wrapped my degree and moved to the UK to join SkyCircuits (now [Callen-Lenz](https://callenlenz.com/)), a company that at the time was developing high-end autopilot for drones. What set us apart at the time was a modular control architecture, on-board script support, and out-of-the-box setup for any platform (multirotors, fixed-wings, helicopters).

We built three user-facing pieces of Ground Control Station (GCS) tools:

* Plan \- mission planning
* Flight \- mission execution
* Toolkit \- a power-user interface to the autopilot

As you probably guessed from the title, the reason we’re here is the last item on the list.

<figure class="center">
   <img src="/images/toolkit/toolkit.png" alt="User interface of SkyCircuits Toolkit">
   <figcaption>SkyCircuits Toolkit</figcaption>
</figure>

SkyCircuits Toolkit was a power user interface that allowed plotting data in real-time (super handy when tuning PIDs) and updating any variable or script on the autopilot. Very useful for all the teams within the company and any power-users that needed it.

I didn’t realize it at the time, but this was a lesson that stuck with me. And 'Toolkit' is such a great name that I always tend to use it.

> “... and we will build a power-user interface exposing all our parameters, and we are going to call it Toolkit”
<p style="text-align:right">Me on at least two occasions with two different clients</p>

## Create software for power-users

Many electromechanical systems require a unique set of parameters that you need to set for each device that you build, no matter how much cookie-cutter-like is your approach. The configuration you may store are PID controller terms, IMU offsets, camera calibration values, device names and so on.

Having worked with over 50 companies, I’ve repeatedly seen software developers involved in provisioning new devices and tuning parameters by hand. This approach is OK when you are prototyping, but once you start scaling up, this becomes an issue.

To illustrate, let’s say the PID controller values of your autopilot are stored in a database, which is the only interface for changing these values. In such a situation, the following request would not be unheard of:

>  Hey Mat, we changed the ESCs on our multirotor and now it’s a little bit wobbly in hover. Can you tune it?

A simple request like this can cost you **30 minutes** at best (that includes the [context switching cost](https://robinweser.com/blog/the-hidden-cost-of-context-switching)). At worst, it might require a field trip and take half a day or more. That’s time your developer isn’t contributing to the product.

By creating power-user interfaces, you can level your robot set-up game between your software, mechanical and electrical teams. Asking a mechanical engineer to learn ROS 2, SQL, or configuration-storage-solution-du-jour is a big ask. Abstracting these interactions into a piece of software that remains stable regardless of the underlying implementation will make all your team equally productive when it comes to robot set-up.

What’s the next step after building your Toolkit? Naturally, automating the entire provisioning process!

