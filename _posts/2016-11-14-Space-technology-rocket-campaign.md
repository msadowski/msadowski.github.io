---
layout: post
title: 2 years ago I launched a rocket (almost) into space - here is what I learned!
description: "Launch much space such rocket into space!"
modified: 2016-11-14
comments: true
tags: [Space, Mechatronics]
image:
  feature: rocketLarge.jpg
---
I've always been fortunate when it comes to finding great learning opportunities. In years 2013-2014 I hit a jackpot, as I spent over a year learning awesome things at Norwegian University of Science and Technology in Trondheim. One of the courses I was lucky to take was Space Technology that was finished by a sounding rocket campaign. Do you want to know what I've learned? Then read on! (There will be a prize for if you make till the end)
<!-- more -->

## The courses
NTNU offers two courses on Space Technology. In order to take [Space Technology II](https://www.ntnu.edu/studies/courses/TTT4235) you have to pass the [Space Technology I](https://www.ntnu.edu/studies/courses/TTT4234) (Or at least those were the rules in 2013). In the first course students learn basic knowledge on Space related subjects ranging from navigation in space to spacecraft thrusters. If you happen to have an opportunity to study at NTNU and take this subject then I highly recommend you go for it! It will also give you a competitive advantage in the Kerbal Space Program!

## Andøya Space Center
<figure class="center">
  <img src="{{site.url}}/images/AndoyaSpaceCenter.jpg" alt="Andøya Space Center">
	<figcaption><a href="https://andoyaspace.no/" title="Andøya Space Center">Andøya Space Center</a></figcaption>
</figure>

The best thing about the 2nd course is that it ends with a sounding rocket campaign at [Andøya Space Center](https://andoyaspace.no/). If you think that this sounds awesome then I can assure you that it indeed is! Andøya is quite far north so as a bonus you get to enjoy polar days and even occasional snow. Did I mention the campaign I attended took place in June?

Andøya Space Center itself specializes in space research and services. You can read more about the work they are doing in [this article](http://www.space.com/34217-andoya-space-center.html).

## The mission
To goal of this whole project was to complete and take part in every aspect of a rocket campaign. We were introduced to all the technical aspects of launching a rocket, rocket aerodynamics, trajectories and stability, ground support equipment etc. We also prepared all the sensors by soldering them onto PCBs and calibrating them.

### The rocket and payload

<figure class="center">
  <img src="{{site.url}}/images/Mongoose-front.jpg" alt="Mongoose 98">
	<figcaption>It's rocket science!"</figcaption>
</figure>

The rocket we used is Mongoose 98. It is almost 3 meters long and has ~10cm diameter. The body structure is made of carbon fiber while the cone is made of glass fibre. If you really want one you can probably get it [here](https://www.madcowrocketry.com/4-98mm-carbon-fiber-mongoose/) but please don't use it at home!

Here is a nifty table summing up selected information about the propellant we used for the rocket:

|A thing        |  A value      |
|:---------|:---------|
|Burnout time | 5.38 s |
|Propellant weight | 6.8 kg |
|ISP ([specific impulse](https://en.wikipedia.org/wiki/Specific_impulse)) | 207.1 s|
|Average thrust | 2562 N |
|Maximum thrust | 3876 N |

### Sensors

<figure class="center">
  <img src="{{site.url}}/images/payload.jpg" alt="Rocket payload">
</figure>

On the rocket we mounted the following sensors:

* photo transistor
* pressure sensor
* two magnetometers (X and Y axes)
* two accelerometers (X and Y axes)
* two temperature sensors (for PCB and the cone)

The photo transistor was used to measure the spin of the rocket (they tend to spin pretty fast in order to achieve gyroscopic stability).

There were two accelerometers used: +/-20g and +/- 50g. One was placed to measure acceleration along X axis (pointing out from the cone), and the second one in Y axis (measuring centripetal force).

## The Launch
Before I jump into the results and conclusions let's see the video of the launch of MUCH SPACE, SUCH ROCKET:

{::nomarkdown}
<iframe width="560" height="315" src="//www.youtube.com/embed/TjDQck4abl8" frameborder="0" allowfullscreen></iframe>
{:/nomarkdown}

### Spin
As I mentioned earlier: some rockets spin to get gyroscopic stability. There are two ways the spin is induced:

1. Spiral launchers
2. Applying spin during flight (by either thruster or aerodynamic forces).

In case of MS Such Rocket the fins had an angle of attack which caused it to spin.

Maximum spin rate we observed was almost 18 rotations per second (almost like your washing machine) and the lowest ~3 RPS.  

### Acceleration
Here is where we made a mistake: we mounted the 20g accelerometer to measure centripetal acceleration and 50g to measure acceleration along X axis. As it turns out the rocket spun quite fast saturating the readings for centripetal force for about 4 seconds and therefore not giving us any results on peak acceleration (at least we know it was over 20g!)

### More data!
The maximum height the rocket reached was 8049 meters at T+37 seconds. At T+88 seconds the rocket hit the ocean about 6 km away from the launch site.

Sadly the two temperature sensors we mounted on the rocket malfunctioned and therefore didn't provide any useful readings. If you know how hot they can get then let me know because I'm super curious!

## Summary
If you have a chance to take part in anything similar I'd advise you to go for it! Taking Space Technology and taking part in this rocket campaign gave me a huge motivation boost and definitely made me a better engineer.

If you happen to be travelling in northern Norway and pass by Andøya I would highly recommend visiting the (Spaceship Aurora)[http://www.spaceshipaurora.no/en]. There you can see some educational videos and take part in space missions!

Taking part in this subject motivated me to take part in ESA's e-deorbit symposium in 2014, but it's a great topic for another post so I will leave you hanging.

## Your special space bonus

<figure class="center">
  <img src="{{site.url}}/images/orbit_x15.png" alt="What if - orbital speed">
	<figcaption><a href="https://what-if.xkcd.com/58/">https://what-if.xkcd.com/58/</a></figcaption>
</figure>

I absolutely love [this](http://what-if.xkcd.com/58/) **what if** answer on orbital speed.

Are you still here? Do you want more? Read our [final report](http://folk.uib.no/kre011/pub/2014%20Report%20of%20the%20student%20rocket%20campaign%20at%20And%F8ya.pdf).
