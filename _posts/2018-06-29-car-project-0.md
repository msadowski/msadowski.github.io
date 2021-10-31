---
layout: post
title: Car Project - Day 0
description: "Recently I've bought some RC car for undefined project work. Here is Day 0 of the project"
modified: 2018-06-29
comments: true
categories: [CarProject]
tags: [RC, Car, DIY]
image:
  feature: car_project/car_4.jpg
---

Recently I had an idea that I would like to play a bit with robotic cars. Here is a day 0 of my car project.

<!-- more -->

## The goal of the project...

Ha!, Got you! There is no goal for the project. I want to keep playing with it, having fun and see what I will arrive to. I might or might not use ROS with it, there might also be some cameras, sensors and extra actuators involved in the future. But one thing is certain - it's going to be fun. So I guess there is a goal after all.

## Setting up

Look at this beautiful **Nssmo Skyllin** body:

<figure class="center">
  <img src="{{site.url}}/images/car_project/car_0.jpg" alt="HobbyKing Mission-D">
</figure>

Now throw it away, we won't need it in this project. What is most important now is what is under the hood

<figure class="center">
  <img src="{{site.url}}/images/car_project/car_1.jpg" alt="R/C car insides">
</figure>

The parts of interest are:
* Servo for steering wit ha rod attached to it that is used for steering
* A DC brushed motor attached to a fairly sophisticated belt system (the car is 4WD)
* An ESC for the brushed motor (there are some switches for reversing the motor direction and it says it accepts 2-3S batteries - something that needs to be investigated in the future)

Sadly the ESC comes with XHT connector which. That leaves me no other choice:

<figure class="center">
  <img src="{{site.url}}/images/car_project/car_2.jpg" alt="Killing the XHT">
  <figcaption>Au revoir to XHT</figcaption>
</figure>

It had to be done because I have plenty of drone batteries that I can utilize and they are all with XT60 connectors. If you don't like soldering or you starting with no batteries I recommend keeping XHT and buying batteries with XHT connectors.

<figure class="center">
  <img src="{{site.url}}/images/car_project/car_3.jpg" alt="XHT crime scene">
  <figcaption>It won't be missed</figcaption>
</figure>

## Testing

At this point I just had two things left to test my setup. Connecting the battery and R/C receiver. After I was done with it this happened:

<figure class="center">
  <img src="{{site.url}}/images/car_project/car_5.gif" alt="Drift car is drifting">
</figure>

## Next steps

There are some further steps I already have in mind for this R/C car:
* Add some sort of controller thing
* Add a plate for holding all the future components (to make sure they don't make their way into the drive train)
* Replace the wheels with some that actually have some traction

Stay tuned for more information!

## Part list

For the sake of completeness, here is what I used in the setup described above:

- Mission-D RC car from [HobbyKing](https://hobbyking.com/en_us/1-10-hobbykingr-mission-d-4wd-gtr-drift-car-arr.html?___store=en_us)
- R/C Transmitter - I used Taranis ([Amazon ref link](https://amzn.to/2KtY2E5))
- R/C Receiver ([Amazon ref link](https://amzn.to/2lIxWTg))
- 2S 2200mAh battery with XT60 connector ([Amazon ref link](https://amzn.to/2Kxy4jj))
- XT60 connectors ([Amazon ref link](https://amzn.to/2Kr4QWf))

The only reason I went with FrSky transmitter and receiver is that I already have them, if I had to start from nothing I would get something cheaper (probably a 2 channel RC car transmitter and receiver). Similarly the reason I got a battery with XT60 connector is that I can reuse the batteries I already have. If you want to start from scratch I would suggest getting a battery with a HXT connector that will match the one on the car (If you want to miss the fun of soldering XT60 connector on the car's ESC that is).
