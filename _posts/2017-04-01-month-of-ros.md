---
layout: post
title: A month of ROS
description: "As a March 1PPM project I decided to focus on learning ROS. The two books I explored this month are: Mastering Ros for Robotics Programming and Programming Robots with ROS."
modified: 2017-04-01
comments: true
tags: [Robotics, ROS, Books]
categories: [1ppm]
---

My March [1PPM]({{site.url}}/1ppm/12-Technical-Challanges/) project significantly differs from all previous ones as I dedicated it to learning Robot Operating System (ROS). 

The two books I _almost_ read during this period are:

- *Mastering ROS for Robotics Programming*
- *Programming Robots with ROS*

In this post I will briefly describe my experience with aforementioned books (which in one particular case was quite short!)

<!-- more -->

# Mastering ROS for Robotics Programming by Lentin Joseph

This is the first book I started going through however after about 100 pages I gave up. There are couple of reasons I gave up on this book; firstly there is lots of repeating information (for example we learn about messages on pages 6, 10, 14, 16). 

Secondly I found many typos and grammar errors which I always find off-putting in books.

Thirdly, my overall impression from reading first 100 pages from the book and a chapter about ROS industrial and movit is that everything is awfully similar to already available ROS tutorials.

Lastly, something completely not to the content. I don't think it's cool for authors to review their own work on sites like [Amazon](https://www.amazon.com/review/R107EG46HUEQ8S/ref=cm_cr_rdp_perm?ie=UTF8&ASIN=1783551798) and giving themselves highest scores, which seems to happen in case of [Lentin Joseph](https://www.amazon.com/gp/profile/amzn1.account.AHS2SXK2LX2W3KNMDTXU3USMMSNQ?ie=UTF8&ref_=cm_cr_rdp_pdp_enth)

<figure class="center">
  <img src="{{site.url}}/images/lentin_review.png" alt="Mastering ROS fro Robotics Programming Amazon review">
	<figcaption>An odd coincidence</figcaption>
</figure>

# Programming Robots with ROS

After being fed up with Lentin Joseph and his Mastering ROS for Robotics Programming I started reading Programming Robots with ROS published by O'Reilly. This time I was way luckier, the book is very consistent and mentions lots of good practices in ROS which is what I kinda need in my life right now.

I think the beginning of the book might be a bit confusing for the complete beginners since sometimes some steps in executing or compiling nodes are omitted (mostly related to changing directories, not specifying where to create files etc.). If someone completed at least beginner ROS tutorials from ROS sites then he won't have any issues, total beginners however might get slightly lost at times.

# Summary

If you are looking to start a journey with ROS then I recommend you take a look at ROS tutorials first. After you nail the structure of ROS packages, messages and build system go read Programming Robots with ROS, you won't regret it.
