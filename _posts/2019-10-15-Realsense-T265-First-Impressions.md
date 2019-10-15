---
layout: post
title: Intel Realsense T265 tracking camera for mobile robotics - first impressions
description: "I have used the Intel Realsense T265 tracking camera for quite a while now. This post summarizes my experience with it and provides some tips on how to use it with ROS."
modified: 2019-10-15
comments: true
tags: [ROS, Robotics]
image:
  feature: t265.jpg
---

In this post I will write about my first impressions after working with Realsense T265 on a wheeled mobile robot and give some tips on the configuration I think is most correct with respect to ROS standard.

<!-- more -->

## Setting up

<figure class="center">
    <img src="{{site.url}}/images/dev_platform.jpg" alt="Mobile robot">
    <figcaption>Robosynthesis development platform used for slam_toolbox evaluation</figcaption>
</figure>

I've tested the T265 on a Robosynthesis Dev Platform that you might have seen it in my [previous post](https://msadowski.github.io/hands-on-with-slam_toolbox/). While working with the T265 tracking camera I spent a fair bit of time going through the documentation and ROS package source code, hope that any of the insights I describe in this post will help you get started.

There are two sets of software we will be interested in in this blog post:
* realsense-ros package ([GitHub](https://github.com/IntelRealSense/realsense-ros)) (while working on this post I was on da4bb5d commit hash)
* librealsense ([GitHub](https://github.com/IntelRealSense/librealsense))

There are some inconsistencies between the above packages and I'll do my best to clear them up in this post. Please note that the T265 seems to be in an active state of development therefore some of the information contained in this post might change over time (I'll do my best to keep it up to date though).

## Coordinate frames

Coordinate frame setup is something I had the most issues with when I first started with T265 with realsense-ros package. First let's look at the T265 frame (as seen in [librealsense](https://github.com/IntelRealSense/librealsense/blob/master/doc/t265.md)).

<figure class="center">
    <img src="{{site.url}}/images/T265_sensor_extrinsics.png" alt="T265 VR coordinate frames">
    <figcaption>T265 coordinate frames</figcaption>
</figure>

T265 uses a VR coordinate system which differs from the one you would expect in ROS. Fortunately the realsense-ros handles the tf transforms for us, making our lives a bit easier. Here is what we need to keep in mind:
* Any frames with a word **optical** in them (e.g. camera_fisheye_optical_frame) are the same frames are the T265 frames in VR coordinates
* The camera and IMU messages are delivered in the optical frames
* The parameter pose_frame_id in the realsense-ros **differs** from the librealsense pose frame. Its orientation follows ROS convention (x-front, y-left, z-up in the global frame)
* pose_frame_id in realsense-ros is the location of the camera in the odometry frame. There is a static transform between the pose_frame_id and the base_frame_id of the camera (not to be mistaken with the base_frame in the ROS traditional sense)

Below you can see an example ROS tf tree that the realsense-ros can provide for us:

<figure class="center">
    <img src="{{site.url}}/images/t265_tf_tree.png" alt="Realsense T265 tf tree">
    <figcaption>slam_toolbox RViz plugin window</figcaption>
</figure>

You should be able to get the same output by setting the following parameters in the launch file (pseudocode):
* camera = "rs_t265"
* tf_prefix = "$(arg camera)"
* publish_odom_tf = "true"
* odom_frame_id = "odom"
* base_frame_id = "$(arg tf_prefix)_link"
* pose_frame_id = "$(arg tf_prefix)_pose_frame"

### Shortcomings

There are two things to note about the above tf structure (most of them coming from [REP-105](https://www.ros.org/reps/rep-0105.html))
* A tf frame can have only one parent. This means that if you have a base_link frame specified in your robot description you won't be able to directly specify a transform for base_link->rs_t265_link or base_link->rs_t265_pose_frame as this would break your tree
* By convention the measurements in the odometry frame have to be continuous (without discrete jumps) this means that if you were to use the setup described above then you would need to set "enable_pose_jumping" parameter to false ([GitHub issue](https://github.com/IntelRealSense/realsense-ros/issues/923#issuecomment-531547065)). More on this later

## The most ROS-proper setup I can think of

Here are some of the considerations for creating a most proper setup with ROS package for Intel Realsense T265 that I can think of.

First of all, I like my sensor frames being relative to the base_link frame of my robot platform. Therefore in my urdf description I would define a static base_link to the camera pose_frame (the ROS one). To have this working we need to set "publish_odom_tf" parameter to false (this way we ensure that the camera pose_frame has a single parent, a base_link).

Say we would like to use the camera as a source of odometry. My suggestion for getting there is to:
* Run a [robot_localization node](http://docs.ros.org/melodic/api/robot_localization/html/index.html) that will listen to the odometry message from T265 and publish an odom->base_link transform. That way we can easily ensure that the tree is continuous.
* Set enable_pose_jumping parameter to false so that the pose of the robot in odometry frame is continuous (**WARNING** read this section until the end before implementing it since it might cause significant errors). Some of the packages you might be using might make some assumptions following REP-105 so better safe than sorry.
* Optional: create a t265 filter node that will change the odometry frame_id from the pose_frame to base_link. That way you should be able to directly compare the various odometry sources on your platform (at least comparing the wheel odometry against the t265 odometry sounds like an interesting experiment)

I successfully run some tests with the first two points from the above list and I was quite satisfied with how 'clean' the setup was w.r.t. ROS good practices. Because of [velocity drift](https://github.com/IntelRealSense/librealsense/issues/4876) that I observed on multiple occasions in my setup I stopped looking into it.

## Wheel odometry

According to [the docs](https://github.com/IntelRealSense/realsense-ros#using-t265) T265 absolutely requires wheel odometry for robust and accurate tracking. To provide the odometry information you will need to:

1. Specify the *topic_odom_in* parameter
2. Create a file with odometry calibration

The first requirement is trivial; you just need to make sure that you correctly specify the topic name.

The fields you need to fill in in the calibration file are T and W vectors in extrinsics field (you will find an example in [this comment](https://github.com/IntelRealSense/librealsense/pull/3462#issuecomment-472491730)). What you need to keep in mind is that T is the translation from camera pose frame (in the sensor/VR frame, **not the realsense-ros pose frame**) to your base_link and W is the rotation between these two frames in **axis-angle representation**. You will find some useful information about this in [this pull request](https://github.com/IntelRealSense/librealsense/pull/3462).

## Closing thoughts

I really like the idea behind the Realsense T265. Having an affordable sensor that can be easily integrated onto any robot would be a great thing to have. I think the T265 is going in the right direction, however I would let it mature before using it on a commercial system but I think it will get there and will provide a true 'plug&play' ROS experience, adhering to good ROS practices.

Is there anything that I missed? Your feedback is highly valued so feel free to leave a comment! I'll be following the T265 development and try to update this post as needed.