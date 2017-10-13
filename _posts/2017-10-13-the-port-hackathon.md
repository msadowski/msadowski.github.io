---
layout: post
title: The Port Humanitarian Hackathon - Report 
description: "The other day I took part in The Port Humanitarian Hackathon where I was working on development of a wearable device for continuous monitoring of Parkinson's Disease symptoms"
modified: 2017-10-13
comments: true
tags: [Hacking, SpesSpirae, CERN]
image:
  feature: port_hackathon.jpg
---

I was very fortunate to be selected as one of the participants of The Port hackathon. In this post you can read about the experience and what our team came up with during one month of preparation and 3 days of hard work.
<!-- more -->

# The Port Hackathon

[The Port](http://theport.ch/) is a Swiss non-profit association cooperating with CERN, whose mission is to bridge NGOs with scientific and technological sectors. Their flagship event is the yearly hackathon during which interdisciplinary teams come together to work on solutions to selected humanitarian challenges.

This year The Port [hosted 7 teams](http://theport.ch/home/the-port-2017/) that spent 60 hours at the CERN idea square working on their concepts and prototypes.

You can find the final presentations from all teams [here](http://cds.cern.ch/record/2288117?ln=en) (our team starts at 38:30).

Contrary to most hackathons The Port Hackathon starts way before participants start hacking. For example my team started brainstorming in September (the event took place 6th-8th of October). Speaking of my team...

## Spes Spirae

<figure class="center">
  <img src="{{site.url}}/images/spes_spirae.jpg" alt="Spes Spirae on stage">
	<figcaption>Most of our team during final presentations</figcaption>
</figure>

I was very fortunate to meet and work with all the people in my team (both those who made it to Geneva and those who supported us remotely). Even though we came from different backgrounds and even continents we "clicked". 

Everyone just knew what to do, we had the same ideas in our heads and  we managed to spend with each other over 50 hours without having any problems or conflicts. And the best part? We came to the idea square as total strangers and left as very good friends.

### Our challenge

{::nomarkdown}
<iframe src="https://player.vimeo.com/video/237453173" width="640" height="360" frameborder="0" webkitallowfullscreen mozallowfullscreen allowfullscreen></iframe>
{:/nomarkdown}

In the above video you can see Lars, who immensely helped us during the hackathon, introducing the motivation behind our challenge. Our goal is to develop a wearable tracker with following objectives:

* Device should be cheap
* All the developed code and designs should be made open source
* Anonymised data from the device should be made openly available if the patient consents to it
* The device should be as simple to use as possible
* Battery life longer than 7 days

During the 3 working days we focused on the following areas:

* Data analysis
* Market research
* Doctor's dashboard
* Hardware for data capture

### Results

Below you can find a video showing Lars' tremor in real time. You can also clearly see 'spikes' when Lars performs tapping or hand grabbing exercises.

{::nomarkdown}
<iframe src="https://player.vimeo.com/video/237972079" width="640" height="360" frameborder="0" webkitallowfullscreen mozallowfullscreen allowfullscreen></iframe>
{:/nomarkdown}

Here are couple of things (in no particular order) that I learned during the challenge:

* 1 in 60 Parkinson's patients are using any monitoring device
* 1 in 60 patients would not agree to share the data from the monitoring device to boost scientific research
* When taking part in hackathon always start with simplest prototype you can and build on top of that
* [serialplot](https://bitbucket.org/hyOzd/serialplot/downloads/) rocks!
* At around 35 hour mark of almost non-stop work your brain might start giving up making it very difficult to focus
* A software that can take data over serial and make FFT (Fast Fourier Transform) in almost real time would be highly appreciated (need to look into it at some point)

# The way forward

We are all in agreement that it would be a great idea to take this project forward. I'm certain I stumbled upon the side project of the century. The fact that it might help people makes it even more exciting.

In the upcoming days I hope we that in the upcoming days we can design the architecture of the watch itself and start coding. 

Stay tuned for more info! If you would like to get involved then you can start with our [github](https://github.com/SpesSpirae organization).

> People overestimate what they can do in a day but underestimate what they can do in a year. - Spes Spirae

