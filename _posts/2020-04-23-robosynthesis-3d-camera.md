---
layout: post
title: "360 camera for industrial inspection with ROS and Robosynthesis robots"
description: "With Robosynthesis we were looking at some 360 cameras for industrial inspection with ROS. This post summarizes our results so far and shows the module we've created in action."
modified: 2020-04-23
comments: true
tags: [Robotics, ROS]
image:
  feature: /360_camera/3d_printed.jpg
---

Together with [Robosynthesis](https://www.robosynthesis.com/) we've been working on developing and integrating a 360 camera module for our ROS based industrial inspection robots. I was so pleased with the results that I couldn't pass on the opportunity to share what we've been up to.

<!-- more -->

## Background

In one of the projects we have worked on we run into an issue - teleoperating a robot in an environment filled with obstacles was really difficult with the cameras we had at the time. In that particular project we ended up with the following camera setup:
* One camera was pointed forward
* Two cameras were pointing to the sides
* Fourth camera was attached at the top of the robot and looking down, allowing the operator to get a good view of what's next to the robot's wheels

The user interface for this setup looked something like this:

<figure class="center">
    <img src="/images/360_camera/ui.png" alt="First iteration UI">
    <figcaption>Not the actual interface</figcaption>
</figure>

Each of the above boxes was providing the user with the camera view. This worked well enough but wasn't very intuitive. Months after that we've started exploring what we've called [a helicopter 360 view](https://www.robosynthesis.com/post/helicopter-360-degree-rgb-camera-view-for-robotic-teleops).

<figure class="center">
    <img src="/images/360_camera/helicopter.png" alt="First shot at helicopter view">
    <figcaption>4 rectified camera images almost ready to be stitched together</figcaption>
</figure>

We have managed to come with the solution that allowed rectifying images from four cameras simultaneously. The idea at the time was that if we get a high enough field of view we should be able to create a nice top down projection of the environment around the robot. There is definitely a value in a top-down projection around the robot but why constrain yourself to 2D view when you can go full 360?

## 360 camera ROS module in action

Before we jump into the technical details of the solution let me show you how it works:

{::nomarkdown}
<iframe width="560" height="315" src="https://www.youtube.com/embed/MlQfebtZFV0" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
{:/nomarkdown}

In this video I'm able to use my mouse to look around the 360 sphere that has the camera feeds overlayed. The interface is very intuitive - you just click and drag the mouse to look at the environment and use the scroll wheel to zoom in or out. There are couple of things I like about this solution:
* Coverage - compared to the alternative solutions we are looking everywhere around the robot at the same time. This is great for inspection because we can record the data and play it back later on allowing us to focus on many points of interest
* Ease of use - this solution is way more intuitive to the operator than our previous 4 separate camera views
* Manageable latency - you can actually operate a robot using this view as the only camera feed

The first thing to get to our solution was to get USB cameras with a high field of view. We ended up with 220 degrees FoV USB cameras. The first test was getting the image feed from a single camera:

<figure class="center">
    <img src="/images/360_camera/first_test.png" alt="First image captured with the high FoV camera">
    <figcaption>Hello World! (That's a proper fisheye view, isn't it?)</figcaption>
</figure>

Then we entered a very rapid prototyping stage with this *handy* prototype:

<figure class="center">
    <img src="/images/360_camera/first_prototype.jpg" alt="First prototype">
    <figcaption>It took about 5 minutes to assemble this one</figcaption>
</figure>

This first prototype allowed me to quickly prove the concept and prepare and test the software stack. In less than a week from the first proof of concept the team took it to the next level:

<figure class="center">
    <img src="/images/360_camera/final_camera.jpg" alt="360 Camera mounted on the robot">
    <figcaption>Final result mounted on the robot</figcaption>
</figure>

In this short period of time we have:

* Designed the casing
* Added a flood light that can be triggered with a ROS service call
* Made it into a module that can be plugged in anywhere on the robot's deck

<figure class="center">
    <img src="/images/360_camera/flood_light.jpg" alt="Floodlight module in action">
    <figcaption>Let there be light</figcaption>
</figure>

<figure class="center">
    <img src="/images/360_camera/flood_light.gif" alt="Animation showing the flood light module">
    <figcaption>Where there is robot there is light</figcaption>
</figure>


## Technical details

I hope that at this point you are wondering how we make this 360 view. The core piece of software that we've used is the [RVIZ Textured Sphere](https://github.com/UTNuclearRoboticsPublic/rviz_textured_sphere) open source plugin developed by Researchers from [the Nuclear and Applied Robotics Group](https://robotics.me.utexas.edu/) at the University of Texas. The plugin takes the image feed from two sources and then applies them onto a 3D Sphere.

<figure class="center">
    <img src="/images/360_camera/rviz_textured_sphere_demo.gif" alt="Nuclear Robotics rviz textured sphere demo animation">
    <figcaption>The original rviz_textured_sphere demo. Credit: Nuclear and Applied Robotics Group</figcaption>
</figure>

Since in the first prototype we have used USB cameras we have used standard UVC drivers for ROS. Unfortunately the cameras we used were suboptimal, providing a rectangular image for a circular image view. Because of that we had to post process the images to be rectangular and the fisheye view to appear in the center.

Since we want the 360 camera view to be used by the operator, the hard requirement for us is a latency below 1 second. In this first prototype we saw around 500ms of latency with the 800x600px resolution for a single camera. In the next iteration I'd like to drive the latency down below 300ms while increasing the camera resolution. Stay tuned for the future updates!

As a bonus please find the below picture showing two [Robosynthesis](https://www.robosynthesis.com/) robots with heaps of modules being ready for industrial inspection.

<figure class="center">
    <img src="/images/360_camera/robosynthesis_robots.jpg" alt="Robosynthesis Robots">
    <figcaption>Modular robots, half of them carrying our 360 camera module</figcaption>
</figure>
