---
layout: post
title: Handle dead nodes in ROS
description: "There are various ways you can get information on the state of the nodes in your ROS based system. This post discusses using 3 different methods to handle zombie or abnormally exited ROS nodes."
modified: 2018-11-10
comments: true
image:
  feature: kinetic_kame.png
tags: [Robotics, ROS, Programming]
---

By default ROS doesn't indicate what is the state of a particular node. If a node exits abnormally (for example when an exception is raised, or a computer running the node is power cycled) ROS master will still hold a reference to this node. Fortunately there are some tools that can help you check/clean the node status.

<!-- more -->

## Background

Couple of moths ago I noticed that ROS master in my setup was holding references to the nodes run on a remote machine, even after it has been shut down. Not being able to trust the list of ROS nodes was far from ideal in my case hence I started looking for a solution. I asked a question on [answers.ros.org](https://answers.ros.org/question/303706/roscore-force-unregister-of-node-that-died/) and was pointed to similar issues: [rosnode zombies](https://answers.ros.org/question/9521/rosnode-zombies/), [ abnormal exited rosnode still be seen on Master?](https://answers.ros.org/question/285530/abnormal-exited-rosnode-still-be-seen-on-master/).

Here are some approaches that I've found out about / I've been pointed towards:

## Brute force cleanup

{% highlight bash %}
rosnode cleanup
{% endhighlight %}

According to [the docs](http://wiki.ros.org/rosnode#rosnode_cleanup) cleanup will purge registration of any nodes that can't be contacted immediately. Depending on your use case this might be suboptimal, if the network interface on a remote machine is down for a second or you cannot contact a node for any other reason you are risking purging a relatively happy node.

## A bond package

Via [ROS wiki](http://wiki.ros.org/bond): A bond allows two processes, A and B, to know when the other has terminated, either cleanly or by crashing. The bond remains connected until it is either broken explicitly or until a heartbeat times out.

In practice I've seen it applied to nodelets. I believe that out of the box 1-1 relationship is supported but maybe, with some clever hacking, it could be extended.

## Ping - node alive

{% highlight bash %}
rosnode ping -a
{% endhighlight %}

[rosnode ping](http://wiki.ros.org/rosnode#rosnode_ping) allows you to, you guessed it, ping a node (or all of them). The nice thing about *rosnode ping* is that it's non-destructive (compared to *rosnode cleanup*). I really like the [node_alive package](https://github.com/tue-robotics/node_alive) that nicely wraps a call *rosnode ping -a* and outputs all the node statuses (either alive, removed, not started or dead) as a [DiagnosticArray message](http://docs.ros.org/melodic/api/diagnostic_msgs/html/msg/DiagnosticArray.html).

There is one potentially big issue with *rosnode ping -a* - it has a timeout of 3 seconds per node (can go up to 30 seconds [before a bugfix](https://github.com/ros/ros_comm/issues/1516) is applied). This brings the following points:

* You can't use node_alive for any critical code that isn't designed to handle the delays in some way
* If you are anticipating to lose many nodes - beware of the delay caused by timeouts (if 5 nodes are out then you are most likely looking at receiving an update from node_alive every 15 seconds + the time it takes to ping the healthy nodes)

## Well architected system

Assuming you have full control over the nodes you use, you can embed a watchdog node in your system that will monitor the state of all the crucial nodes and act accordingly in case you establish any one of them timed out.