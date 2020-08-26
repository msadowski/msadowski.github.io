---
layout: post
title: "Thoughts on 2 Years as a Remote Robotics Consultant"
description: "Earlier this year I hit a milestone of working 2 years as a remote robotics consultant. This post sums my experience and shows the approach to consulting that I've taken"
modified: 2020-08-26
comments: true
tags: [Robotics]
image:
  feature: M_consulting_cropped.png
---


It’s that time of year again when I’m thinking back on the journey I set up for myself by deciding to go into Robotics Consulting. This article is a follow up to the blog post I wrote [last year](https://msadowski.github.io/one-year-of-robotics-consulting/). This year, I’ve decided to try and cover the most important aspects of what I’m doing, and how it has been working out for me.

<!-- more -->

## The work I had

I’m very grateful for the kind of work that I do - it feels amazing doing what I love and getting paid for it. I would never change my work for anything else. Every single one of my clients has an interesting problem to solve, and I’m yet to get my first negative experience working with someone. Looking back on all my clients, I’ve worked with many interesting people all around the world, mostly helping them to develop software for mobile robots and drones.

<figure class="center">
    <img src="/images/2_yr_consulting/clients.png" alt="Map with 22 Clients I had all over the world">
    <figcaption>A world map of the 22 clients I've worked with over the past 2 years</figcaption>
</figure>

<figure class="center">
    <img src="/images/2_yr_consulting/clients_europe.png" alt="Image showing 8 of my clients in Europe">
    <figcaption>My European clients</figcaption>
</figure>

The duration of the projects I take on are between half a day and 2.5 years long. Everything depends on the client and what they require. Most of my projects kick off with a consultation - helping the client to solve a particular problem by sharing my expertise and experience. After this initial phase, some of the clients decide to hire me as a contractor, helping their technical team solve the challenges they face, while others are happy to carry on working by themselves with the advice received.

As a tangent to the consultations, I also help evaluate robotics funding proposals, as a reviewer in EU technological funding projects [ESMERA](http://www.esmera-project.eu/welcome/) and [TRINITY](https://trinityrobotics.eu/), making sure your tax euros are spent on truly cutting-edge projects.

### Putting myself in my clients’ shoes

Every project I take on, I treat as my own. This results in me trying to solve issues before they happen, and sometimes even pushing back on client’s decisions - “How about we don’t remove this safety feature?”. Another side effect of treating the project as my own is that I put the client first, before my own business interest, meaning I might earn less for the project overall.

As an example, I was hired to test the idea a client had for a robotic project. The idea turned out to be borderline feasible, with currently available technology, but potentially requiring lots of R&D work. If I sold the project as “no problem, it’s totally doable”, I would definitely get more work supporting the R&D work required. By putting the project and client’s interests first, however, we finished the project after the feasibility study, having realised the extent of R&D work needed.

Since terminating such a project before going to an R&D phase is something I would do, I’m content and the client is most likely even more so, since they don’t have to spend lots of money, not understanding the R&D effort required. Instead, they can decide whether to pursue the idea further. If the client does decide to take a risk, they know that I’ll be there, ready to treat their project as my own, and providing honest advice, even if it means I get less work from it.

### The hardware

Working with hardware is one of the most enjoyable parts of my job. This year, I’ve been focusing a lot on RTK solutions (I have one blog post in the works on a neat RTK setup). Also this year, I finally got my hands on a multi-plane LiDAR, something I’ve been very keen on ever since Velodyne revealed their first multi-plane units.

<figure class="center">
    <img src="/images/2_yr_consulting/hardware.png" alt="Hardware units I've worked with">
    <figcaption> Some of the hardware I've worked with since I started working as a Remote Robotics Consultant</figcaption>
</figure>

## The Work I Did Not Win

### The dream project

At one point, I was approached by a client who wanted to create a certain type of robot that I’ve always dreamt of working with. The task would require me to liaise with manufacturers and developers to get the robot ready, and enable support for autonomous navigation in various terrain types. In short, a hugely challenging project that would immensely help me grow, whilst being super fun at the same time. When discussing the project in more detail, I noticed that the budget for this client was not an issue - any platform for any price was OK, as long as it met the requirements. However, when asking follow up questions, it became clear that the project target was to equip robots with weapons.

<figure class="center">
<video controls="controls" class="center" style="width:100%">
    <source src="/images/2_yr_consulting/bd.mp4" type="video/mp4">
</video>
<figcaption>The kind of project I’m not keen to work on. Source: <a href="https://www.youtube.com/watch?v=y3RIHnK0_NE">Bosstown Dynamics</a> (Corridor Digital)
</figcaption>
</figure>

When I was applying to University, I promised myself I would not work on weapons, even if these projects have a virtually unlimited budget. I figured that even if I was to fold my consultancy, I would rather do that than compromise on my values.

### The bargain

In the two years working as a Robotics Consultant, I’ve noticed a pattern: the more a potential client bargains before starting the job, the more bargaining and complaining will follow, resulting in an unpleasant experience for everyone involved. In the same bucket as bargaining clients are the UpWork projects that want you to create a SLAM library from scratch for a fixed price of $50. These days, I tend to reject these types of clients and contracts straight away, saving everyone’s time.

### Watch out for the legal stuff

If I’m receiving 25 pages of legal documents to review before engaging with a customer, it’s a  potential red flag. I don’t mind reading legal documents, but I would rather not involve a lawyer before even starting a project. For me, a huge red flag is when a potential client adds a very vaguely described 3-year non-compete clause, covering a whole industry. I find such terms too restrictive and would never take a project on like this, unless the project also paid for a 3 year holiday after wrapping everything up!

### The lack of expertise

As much as I would love to know everything about Robotics, it’s not likely to happen anytime soon. I will never take on projects that fall outside of my expertise (for example, algorithms for soft robotics), unless the client is persistent and understands an R&D process. My go-to advice in such cases is that it’s not feasible to pay my high rates for me to catch up on the topic. In a hypothetical situation, if the project falls outside of my expertise, but it’s in an area I’m planning to pursue, I’d offer a very low rate for the project, highlighting that I will need to do some catching up.

## Remote Robotics

People often ask me how I work on robotics projects remotely (I’ve been doing this since day 1 of my consultancy). When I’m doing some high level work, it’s usually not an issue. It only becomes slightly more problematic with hardware-oriented projects. The options I’ve tested so far are:

* For ROS systems, working with bag files, and in the case of drones, working with log files
* Clients shipping the hardware to me
* Travelling to the client’s location
* Remote control

<figure class="center">
    <img src="/images/2_yr_consulting/office.jpg" alt="My office in the times of pandemic">
    <figcaption>Me and my partner's office in the time of the COVID-19 pandemic</figcaption>
</figure>

ROS is absolutely the best thing that happened to people like me - I can easily review things online and even develop software with just [bagfiles](http://wiki.ros.org/rosbag). However, it’s not always feasible to work with bags, especially if you need to do some work related to sensors and actuators. In these cases, clients will often ship their hardware to me, so that I can integrate it locally. A very convenient way to work with clients in this way is to use a temporary import - it saves you the risk of paying taxes and duties for the expensive robotics parts, and as far as I know when doing a temporary import, you can keep the items for up to a year.

Travelling to the client’s location usually results in a couple of days of a hackathon solving issue. I love these, as it allows me to put the faces to Slack usernames, and I like the dynamics of these kinds of projects. On the flip side, the last time I did it, I ended up doing about 60 hours of work in 5 days - not very feasible, especially as you need to rest for a couple of days afterwards, but I would do it again!

Remote control of the client’s desktop is something that I try to avoid as much as possible when working with hardware. I find that not having physical access to the components is often very limiting (“can you please unplug the cable for me?”) and I estimate that in the worst cases, you are 20-40% less productive by working on robotics in this way.

## Other Thoughts on Consultancy and Self-Employment

### Freedom

Freedom is one of the most important aspects of what I do. If I don’t work with hardware for a project, then I can usually work from whatever place I want. If I have a day when I don’t feel like working, or can’t work, then taking a day off is not an issue. To an extent, I can choose whichever projects to work on (or at least to which I can say “no” to).

<figure class="center">
    <img src="/images/2_yr_consulting/outdoors.jpg" alt="A mountain view">
    <figcaption>Having a possibility to go out into the mountains in the middle of the week is something I appreciate a lot</figcaption>
</figure>

This freedom, however, comes with a price attached to it - if I’m not working, then I’m not making money. I don’t get bank holidays and if I get sick and can’t work, then I’m not earning either. These are the main reasons why I think working for oneself is not for everybody, especially when having no projects lined up, as it can become quite stressful.

Being self-employed means that you are at the centre of your business. For me, this means that my physical and mental health are my number one priorities. Examples of how I exercise these are:
* Three times a week, around 4pm, I do sports (normally, I go to the gym, but during the pandemic, I’ve been going for a run)
* Practising meditation as the first thing I do in the morning
* In case of health issues, I look to fix them without thinking twice about the money (being located in Europe is a huge advantage here as I know no doctor’s visit will ruin me, financially)

### Upwork

Over the past year, most of my work has come from Upwork (you can see my profile [here](https://www.upwork.com/freelancers/~0196b3ccb97605e632)). I won’t repeat the things I said [last year](https://msadowski.github.io/one-year-of-robotics-consulting/#upwork) - I’ve grown to appreciate the service. I realised that the 20% fee for the first $500 doesn’t hurt as much when you target long-term or high bid projects. The value I get in terms of not having to chase customers for payment, easily logging the time, and the discoverability, makes it worth it in my case. Some people criticise Upwork for the screenshot it takes of the screen while you are working for hourly projects, but for me, this is a non-issue - when I’m working on the project, I’ll never have anything private open and I’ll be just doing the work I charge customers for. If a project required me to have a webcam on though (it’s an optional requirement I believe), I wouldn’t accept it.

<figure class="center">
    <img src="/images/2_yr_consulting/top_rated_plus.png" alt="UpWork Top Rated Plus badge">
    <figcaption>My hard-earned Upwork Top Rated Plus badge</figcaption>
</figure>

If you are looking into starting on Upwork, then I’d recommend figuring out the best way to get paid, without losing out on payment and currency conversion fees. The most cost-efficient way I came across was setting up a [TransferWise account](https://transferwise.com/invite/u/mateuszs27) (<-- with this link, you’ll get your first transfer of up to 500 GBP for free). In TransferWise, I’ve set up a US account allowing me to handle ACH payments (the method Upwork uses to pay me). That way, I don’t pay anything to get the money from Upwork, but instead, I only pay for the TransferWise exchange rate, which is the best I found when I was looking into this.

To save time on browsing Upwork projects, I set up an RSS feed with the keywords that interest me. That way, I can see all of the projects that were posted on the platform, without having to go through the same ones by visiting the website.

### Mastermind group

For over a year now, I’ve been having weekly [Mastermind group](https://en.wikipedia.org/wiki/Mastermind_group) meetings with my friend, Michał (if you need to tell a visual story, then I highly recommend [getting in touch with him](https://www.michalkalina.com/)). In these weekly calls, we discuss the business and problems we are facing. Having someone to bounce ideas around is very helpful, especially if you can get honest feedback from someone running into similar issues as you do.


<figure class="center">
    <img src="/images/2_yr_consulting/mastermind.png" alt="Me and Michał during one of our Mastermind meetings">
    <figcaption>Me and Michał during one of our Mastermind meetings. Do you also think outdoor business meetings should be the norm?</figcaption>
</figure>

### Attention switching penalty

This is advice to all of you working on multiple projects at the same time. In my experience, every time I switch focus from one project to another, it takes about 10-20 minutes to get into the zone, and to not think about the other one. Because of this, I try to switch between projects as little as possible - ideally less than 2-3 times a day. Most of the time before a switch, I try to take a short break.

## Closing thoughts

Having worked for over 2 years as a Remote Robotics Consultant, I can’t imagine doing anything else. I take lots of satisfaction and pride in my work, and I hope clients notice my focus on uncompromised quality. There is a small issue though - if your work is your hobby - when do you rest? I realised I might have been pushing myself a bit too hard during lockdown (having a clear split between the office and home was probably one of the reasons I was working from the office in the first place). Looking out into the future, I’m hoping to rent a proper workshop and work from there.

Do you have any questions about my work? Feel free to leave a comment and I'll answer it! And if you would like to work with me, then feel free to send me an [e-mail](mailto:contact@msadowski.ch)
