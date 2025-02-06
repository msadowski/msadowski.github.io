---
layout: post
title: "A Roboticist Visits FOSDEM 2025"
description: "In February 2025, we organized the first Robotics devroom at FOSDEM. This blog post describes my experience as a first-time FOSDEM attendee and devroom organizer."
modified: 2025-02-06
comments: true
tags: [Robotics]
image:
  feature: fosdem2025/header.png
---

## Intro

This year, I paid my first visit to FOSDEM (Free and Open Source Software Developers' European Meeting) and helped organize the Robotics and Simulation Devroom. This blog post summarizes my experience.

<!-- more -->

Last year, Kimberly McGuire approached me about organizing a robotics devroom at FOSDEM. One thing led to another, and we assembled a dream team of organizers for the "Robotics and Simulation Devroom":

* [Arnaud Taffanel (Bitcraze)](https://www.linkedin.com/in/arnaud-taffanel-a5750211)
* [Fred Gurr (Eclipse Foundation)](https://www.linkedin.com/in/fred-g-5388a311/)
* [Kimberly McGuire (Independent)](https://www.linkedin.com/in/knmcguire/)
* [Lucas Chiesa (Ekumen)](https://www.linkedin.com/in/lucaschiesa/)
* [Mat Sadowski (me)](https://www.linkedin.com/in/mateuszsadowski/)

<figure class="center">
    <img src="/images/fosdem2025/organizers.jpg" alt="Devroom organizers">
    <figcaption>Organizers of the Robotics and Simulation Devroom</figcaption>
</figure>

and just like that, I set off for a [Club Mate](https://en.wikipedia.org/wiki/Club-Mate#Culture)-fueled endeavor where hackers meet open-source—or should I say, make open-source possible.

What sets FOSDEM apart from other conferences I attended in the last couple of years is the fact that the event is free to attend. I think it does not get more open-source than this.

## The Robotics and Simulation devroom

<figure class="center">
    <img src="/images/fosdem2025/fosdem_header.jpg" alt="Header Image of our Devroom">
    <figcaption>Graphics for our room by <a href="https://www.michalkalina.com/">Michał Kalina</a></figcaption>
</figure>

Our room hosted the first Robotics devroom in FOSDEM's history. In the last section of this blog post, I'll provide a list of robotics-related talks from previous FOSDEMs. We had a room for a half-day and managed to squeeze in 14 talks in less than four hours. This is largely due to many presenters agreeing to turn their talks into lightning talks. The room was full for most of the time, with people occasionally queuing outside to enter.

## The talks I found interesting

Below is a short list of talks that I found interesting and attended:

* [The whole Robotics and Simulation track](https://fosdem.org/2025/schedule/track/robotics/)
* [SatNOGS-COMMS: An Open-Source Communication Subsystem for CubeSats](https://fosdem.org/2025/schedule/event/fosdem-2025-6024-satnogs-comms-an-open-source-communication-subsystem-for-cubesats/) - stepping outside of my comfort zone and going more embedded. Some very interesting lessons learned about using Zephyr-RTOS
* [The road to open source General Purpose Humanoids with dora-rs](https://fosdem.org/2025/schedule/event/fosdem-2025-5525-the-road-to-open-source-general-purpose-humanoids-with-dora-rs/) - very nice demo with Pollen Robotics Reachy robot and, hopefully, a step towards general robots
* [Exploring Open Source Dual A/B Update Solutions for Embedded Linux](https://fosdem.org/2025/schedule/event/fosdem-2025-6299-exploring-open-source-dual-a-b-update-solutions-for-embedded-linux/) - good overview on strategies for reasonable firmware update workflows and overview on whats out there
* [Discovering the Magic Behind OpenTelemetry Instrumentation](https://fosdem.org/2025/schedule/event/fosdem-2025-4146-discovering-the-magic-behind-opentelemetry-instrumentation/) - trying to expand my horizons on logging. Implementing this with ROS would make a really nice side project
* [What can PyArrow do for you - Array interchange, storage, compute and transport](https://fosdem.org/2025/schedule/event/fosdem-2025-6092-what-can-pyarrow-do-for-you-array-interchange-storage-compute-and-transport/) - I've heard of Arrow quite a bit lately and I'm excited to look more into its IPC capabilities and zero-copy approach. Did you know that with Arrow you can seamlessly switch between Numpy, Pandas and other libraries with zero-copy using Arrow?
* [Programming ROS 2 with Rust](https://fosdem.org/2025/schedule/event/fosdem-2025-6548-programming-ros-2-with-rust/) - Julia made a nice presentation in the Rust room on using ROS 2 with Rust
* [Lessons from rewriting systems software in Rust](https://fosdem.org/2025/schedule/event/fosdem-2025-5088-lessons-from-rewriting-systems-software-in-rust/) - Some interesting insights on rewritting software that I believe does not only apply to Rust. One point that drove it home was depending on other's people code

<figure class="center">
    <img src="/images/fosdem2025/reachy.jpg" alt="Reachy Robot by Pollen robotics">
    <figcaption>Reachy used for a demo in the dora-rs presentation</figcaption>
</figure>

Here is a list of talks I didn't get to see but that I'll catch-up on once the videos are available:

* [Zephyr RTOS Roasting Party](https://fosdem.org/2025/schedule/event/fosdem-2025-5760-zephyr-rtos-roasting-party/)
* [Using embedded Rust to build an unattended, battery-powered device](https://fosdem.org/2025/schedule/event/fosdem-2025-6300-using-embedded-rust-to-build-an-unattended-battery-powered-device/)
* [Hugging Face ecosystem for Local AI/ ML](https://fosdem.org/2025/schedule/event/fosdem-2025-6341-hugging-face-ecosystem-for-local-ai-ml/)
* [Apache Arrow: The Great Library Unifier](https://fosdem.org/2025/schedule/event/fosdem-2025-4801-apache-arrow-the-great-library-unifier/)
* [From Supercomputer to Raspberry Pi: Building Open Source Polish Language Models](https://fosdem.org/2025/schedule/event/fosdem-2025-6660-from-supercomputer-to-raspberry-pi-building-open-source-polish-language-models/)
* [Lessons From 10 Years of Certifying Open Source Hardware](https://fosdem.org/2025/schedule/event/fosdem-2025-4257-lessons-from-10-years-of-certifying-open-source-hardware/)
* [Augurs: a time series toolkit for Rust](https://fosdem.org/2025/schedule/event/fosdem-2025-4668-augurs-a-time-series-toolkit-for-rust/)
* [Building a watt-meter esp-rs and a rocket backend](https://fosdem.org/2025/schedule/event/fosdem-2025-5470-building-a-watt-meter-esp-rs-and-a-rocket-backend/)
* [Automating Low-Level Firmware Validation with Robot Framework](https://fosdem.org/2025/schedule/event/fosdem-2025-5996-automating-low-level-firmware-validation-with-robot-framework/)
* [Infra for Drones: Lessons learned from 15 years of open source robotics](https://fosdem.org/2025/schedule/event/fosdem-2025-6384-infra-for-drones-lessons-learned-from-15-years-of-open-source-robotics-/)
* [RTCP, Racecars, video and 5g](https://fosdem.org/2025/schedule/event/fosdem-2025-5586-rtcp-racecars-video-and-5g/)

## Tips&Tricks

### Want to be a speaker? Make a quality proposal

<figure class="center">
    <img src="/images/fosdem2025/robbie.jpg" alt="An image of a Robot Leaflet">
    <figcaption>Robbie developed by Kimberly to bravely promote our devroom during the event</figcaption>
</figure>

This year, while organizing the Robotics and Simulation devroom, we received 24 proposals and had to make tough choices to fit them into half a day. We selected only the best and asked some speakers to condense their presentations into 5-minute lightning talks.

Some advice for next year's applicants:

* Make sure your talk is related to open-source or is of great interest to community
* If your talk involves robots or robotic operations, your proposal will be much stronger if you are already beyond the simulation stage and have deployed things on real hardware
* Make a super high-quality proposal. You are competing for a speaking slot with world-class roboticsists, a two-sentence abstract won't cut it, no matter the reputation of your project
* Include links to your repository. It helps if it's actively maintained
* Don't use AI to write your proposal (reviewers can often tell, and you get huge minus points for that)

### Accomodation

I booked the accomodation within walking distance to the venue thinking it makes perfect sense to be as close to it as possible. It turns out the right question to ask yourself is: are you going to hang out with someone after the event and if you do, where will it happen and would you prefer to commute early in the morning or at night.

On Friday and Saturday I ended up spending the evenings near the city centre of Brussels with my fellow co-organizers and had to commute back at night towards the campus. In hindsight, I would prefer to sleep closer to the center and get to campus early.

### Making it to see the talks

Often, the rooms are packed! If you find yourself going to a talk that might be popular, you get inside the room earlier. Once the rooms are full, you are not supposed to enter them for safety reasons.

### Taking notes

When we organized the room, we asked the speakers to upload their slides to Pretalx. This meant that the slides showed up in the talk description and as an attendee you could download it and make notes right there on the slides. If I just had my tablet with me!

## List of robotics-adjecent talks throughout FOSDEM history

2016
* [Simulating Humanoid Robots in the Cloud: the testing behind the biggest world competition](https://archive.fosdem.org/2016/schedule/event/testing_robots_in_the_cloud/)

2017
* [Loco Positioning: An OpenSource Local Positioning System for robotics](https://archive.fosdem.org/2017/schedule/event/loco_positioning_crazyflie/)

2018
* [How to build autonomous robot for less than 2K€](https://archive.fosdem.org/2018/schedule/event/autonomous_robot/)
* [Rusty robots; Programming a self-balancing robot in Rust](https://archive.fosdem.org/2018/schedule/event/rusty_robots/)

2020
* [ROS2: The evolution of Robot Operative System](https://archive.fosdem.org/2020/schedule/event/ema_ros2_evolution/)
* [Making an IoT robot With NuttX, IoT.js, WebThings and more](https://archive.fosdem.org/2020/schedule/event/iotnuttx/)
* [Introduction to Eclipse iceoryx; Writing a safe IPC framework for autonomous robots and cars](https://archive.fosdem.org/2020/schedule/event/ema_iceoryx/)

2023
* [FOSSbot: An open source and open design educational robot (Lightning Talk)](https://archive.fosdem.org/2023/schedule/event/fossbot/)

2024
* [Controlling a 6 degree Robot Arm using a 48K ZX Spectrum](https://archive.fosdem.org/2024/schedule/event/fosdem-2024-2898-controlling-a-6-degree-robot-arm-using-a-48k-zx-spectrum/)
* [Dora-rs: simplifying robotics stack for next gen robots](https://archive.fosdem.org/2024/schedule/event/fosdem-2024-3225-dora-rs-simplifying-robotics-stack-for-next-gen-robots/)


These are all the thoughts I have on FOSDEM 2025. See you in Brussels next year!