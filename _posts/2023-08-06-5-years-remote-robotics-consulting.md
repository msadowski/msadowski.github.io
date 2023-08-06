---
layout: post
title: "Remote Robotics Consulting - A Five-Year Retrospective"
description: "In this blog post, I will highlight some of the updates since my last blog and offer some advice that I hope will be useful for anyone looking to get into technical consulting."
modified: 2023-08-05
comments: true
tags: [Robotics]
image:
  feature: M_consulting_cropped.png
---

## Intro

If you've been following my blog for a while, you might have seen my earlier blog posts summarizing my experience as a Remote Robotics Consultant. In case you’d like to start from the beginning, here is the ordered list:

* [One Year of Working as a Robotics Consultant](https://msadowski.github.io/one-year-of-robotics-consulting/)
* [Thoughts on 2 Years as a Remote Robotics Consultant](https://msadowski.github.io/2-years-remote-consulting/)
* [Remote Robotics Consulting - 3.5 Years In](https://msadowski.github.io/3-5-years-remote-consulting/)

In this blog post, I will highlight some of the updates since my last blog and offer some advice that I hope will be useful for anyone looking to get into technical consulting.

Strap in, and let’s go!

<!-- more -->

## Update on my Previous Blog Post

Last time, I mentioned multiple things that are now out of date:

* I no longer rent out a workshop. Focusing more and more on software projects, I practically stopped using it. The 30-minute commute one way did not help either, so I resigned from it at the beginning of this year.
* Currently, approximately 2% of my projects come from Upwork. They are only short-term consultations. The rest are direct projects with clients.
* I solved the context-switching problem - I haven’t had issues with it since my last post, so that's great news. What helped here was turning off all notifications. I found my e-mail the most distracting and have since decided to check it at my own pace.
* Work-life balance? - Here, I fell behind compared to the last update. On two occasions, I was edging on burnout. Things are better now, but I realized I don't have hobbies outside of robotics. I’m actively working on this and starting to see the light at the end of the tunnel.
* In my last update, I mentioned taking on all opportunities that fell into my lap. Well, not anymore. I’ve become more selective and more mindful of my schedule.

<figure class="center">
    <img src="/images/3_yr_consulting/office_collage.png" alt="Collage of the office">
    <figcaption>Bye-bye workshop.</figcaption>
</figure>

### Passive Income - Here I Come!!

Last year, I released a Manning Live Project series on [Building Mobile Robots with ROS 2](https://www.manning.com/liveprojectseries/build-mobile-robots-with-ROS2?utm_source=mateusz&utm_medium=affiliate&utm_campaign=liveproject_sadowski_build_5_6_22&a_aid=mateusz&a_bid=a308c7c4). Since it’s been released, around 230 students have enrolled in it. This project took me slightly over a man-month of time, invested over six months. Since I'm about to earn my advance, earning a month's consulting salary from this project will take approximately 19 years. Even though I’m not earning a huge amount from this project, if I could go back in time, I would do it again. It’s been an incredible journey, and I think it’s time to look into publishing more content like this. Lately, I’m finding writing quite rewarding.


## Weekly Robotics

Three months after I became a consultant, I started [Weekly Robotics](https://www.weeklyrobotics.com/), a newsletter that sums up the most exciting research, projects, and news I have come across during my industry research. The newsletter is now closing on 12,000 subscribers on two delivery channels.

<figure class="center">
<video controls="controls" class="center" style="width:100%">
    <source src="/images/5_yr_consulting/wr_features_2022.mp4" type="video/mp4">
</video>
<figcaption>Header images of all Weekly Robotics issues in 2022.</figcaption>
</figure>

2022 was a terrific year for the newsletter, where we had many advertisers, started the first long-term partnership with AMD, and managed to budget about $10k for its growth and newsletter content.

I've put this money towards:

* [Visiting ICRA this year](https://www.weeklyrobotics.com/weekly-robotics-250)
* Getting technical folks to write long-term articles
* Doing [GitHub sponsorships](https://github.com/orgs/WeeklyRobotics/sponsoring) for some open-source robotics developers
* E-mail distribution, hosting, etc.
* Subcontracting sponsor outreach
* Article editing

The newsletter, however, is far from profitable, especially if I’m not spending this time on consulting. Last time I checked, the time spent on this project annually is about one man-month. It also does not bring in many projects (I’ve counted three since I started it). This is understandable, though, as I’m not advertising my consultancy there and prefer it to be separate from my branding.

The best thing about the newsletter now is the community. Our [Patreon](https://www.patreon.com/WeeklyRobotics) slack channel has grown quite a bit, especially since I started inviting non-Patreon users. Having a small community to bounce ideas off of and discuss robotics and projects is great!


## Insights

These blog posts are helpful to many (or so I’ve been told), so let's jump into the meat of it. Technical consulting is not a zero-sum game, so I’m more than comfortable sharing what has worked for me on this journey. I hope some of you will find these points helpful.


### My Problem-Solving Strategy

If you have clients who are no experts in robotics, as a consultant, you can have a big leverage on them. Let’s consider an exaggerated situation where a client wants to move an item from point A to B and they know nothing about automation. To drive the point, let’s say that you either:

1. Propose to develop a ground vehicle with a manipulator arm attached
2. Tell them the conveyor belt exists and to check them out

Approach a) would provide you with lots of work (and money), while approach b) would either mean that you don’t engage in the project or you take the role of an integrator. Maybe I’m a bit too idealistic, but I always use the latter approach and put my customer’s interest first. I think it’s much easier to work on projects you believe in, and you sleep better at night, too!


### Two Rooms

There is a massive overlay between engineering consulting and consulting in the creative fields. In his book, [The Business of Expertise](https://www.expertise.is/), David Baker presents a concept of [two rooms](https://www.davidcbaker.com/your-creative-firm-is-a-single-building-with-two-rooms) that I, of course, took over and incorporated into my strategy.

<figure class="center">
    <img src="/images/5_yr_consulting/image1.png" alt="Startegy and execution rooms">
    <figcaption>The two-room concept. Source: <a href="https://www.davidcbaker.com/your-creative-firm-is-a-single-building-with-two-rooms">davidcbaker.com</a></figcaption>
</figure>


In my work, I have two hats:

* Consultant - providing high-impact and high-value advice. Usually, this is quite high-level (strategy in the image above)
* Contractor/Freelancer - very hands-on role, implementing hardware and software to solve customer problems (execution in the image above)

If you start with a client providing strategy-related work, moving to execution is relatively easy and can be a part of your package. If you begin with execution, it might be tough to start wearing a strategy hat. You’d need a mix of luck, good salesmanship, and someone inside the organization recognizing your value.


### Shut Up and Listen

When listening to a client, you might feel that you know how to fix their issue and that it is straightforward. Even though the urge to interrupt might be strong, you should not jump to conclusions. Chances are, your client has worked for years in their industry, and they know it way better than you ever will. So strap in, shut down your eagerness to propose solutions, and ask good questions instead.

<blockquote>
    <p>Client: … and the UGV we are working on needs to be localized to within 1 meter in the target environment.</p>


    <p>You: Easy-peasy, we will take a LiDAR, put some SLAM library on this baby and it’s done.</p>


    <p>Client: and the target environment for this client is a mirror factory.</p>


    <p>You: …</p>
</blockquote>


<figure class="center">
    <img src="/images/5_yr_consulting/image4.png" alt="Shocked Pikachu meme">
    <figcaption>You, after rudely interrupting a client with your solutions, instead of letting them finish describing their problem in detail…
</figcaption>
</figure>

### Don't Do the Price Dance

Sometimes, you'll get approached about a project. You will exchange a couple of e-mails, then you'll take a 30-60 minute unpaid call, then you'll create a summary of the meeting, and then, you'll gently start hinting towards talking about money. If you do this, then I hope it works for you. Chances are, you've wasted everyone's time (most importantly, yours).

What I tend to do these days is reply to the first message along the lines of:

<blockquote>
    <p>Hi!</p>

    <p>Thanks for your message. What you describe sounds like a project I could help with. I charge my clients on an hourly basis at $X. If this works for you, I would love to have an introductory call and take it from there.</p>
</blockquote>

Quite often, this will be the last interaction with a prospective client. Some people will also get offended if you discuss the price too early. 'Godspeed' is what we tell these folks and we move on.

The clients worth working with don’t question my hourly rate and respect my expertise, so it’s always worth being upfront about your fees.


### Charge by the Project if You Want to Be Rich.

Jonathan Stark has [a blog post](https://medium.com/@jonathanstark/how-i-realized-that-hourly-billing-was-nuts-2aee1fa959b3) and even a book series on how projects, where you bill by deliverables, are superior to hourly-based ones. Working like this is comfortable for your clients because they know how much they will pay upfront, and for you because you'll incorporate a hefty premium in your estimate, and it will all work out, right?

Well, maybe. I find it e-n-o-r-m-o-u-s-l-y difficult to price R&D projects because it's all fine and well until you suddenly need to recompile a Kernel of the onboard computer, or it turns out that a fried electronic component caused a bug you've been tracking for three days or the customer has changed requirements. Now you need to explain how this eats into the budget and figure out how to charge extra and get paid.

I think fixed-price projects work if you have a good idea of what you are doing. This means you have experience with most of the hardware the client wants, or you are in a position to choose for them, and you know roughly how to solve the software part, and ideally, you have delivered similar software/hardware in the past.


### Remote Hardware Work Overhead

Working remotely on hardware (e.g., integrating a LiDAR, IMU, or some SBC) takes 2-3x longer without physical access. Because of that, I usually advise clients to ship hardware to me or get me to their location. For purely remote work, it's best when the client has a local team that can do the hands-on work and gather data for you, following your instructions.


### Don't Withhold Expertise

Imagine a very theoretical scenario of a conversation between (Y)ou and two consultants (C1, C2) during an introductory call:
<blockquote>
    <p>Y: We need help with our localization stack. For some reason, our robot position estimate is not turning in the global frame when it’s rotating just fine in real life.</p>


    <p>C1: Oh, that's not ideal. I've set up the EKF on hundreds of platforms. If we work together, I'll be able to fix this.</p>


    <p>C2: This is an interesting problem. If I were you, I would check which axes are enabled in the EKF and ensure that the yaw axis of IMU or the angular velocity from wheel odometry is fused. If enabled, I would double-check the covariances in your sensor messages.</p>
</blockquote>

Which consultant would you pick to work with? Using the second approach has been working well for me so far. I would not worry about a hypothetical scenario of a client taking your advice and running away. It will happen occasionally to C1 and C2.

Even though it might be far from reality, C2 can appear more knowledgeable during the call because C1 didn’t showcase their skills. Having some portfolio can help C1, but after the call, C2 already contributed value to the company, and they might be more keen to hire them.


### Consider Regulations

If you have this grand vision of developing the slickest delivery drone or a new robot for eye surgery, and your vision is very clear (pun intended!), and you want it to be on the market in half a year, I might have some bad news. Some fields are highly regulated, and for a good reason. Regarding drones, you should look at FAA certification in the U.S. or EASA in the EU.

My answer in these situations is usually: “Great, let’s do it. I can help you build the prototype, but let’s get onboard someone who knows the regulations so that we start building with these in mind”.

If you disregard the certification, you might create a prototype that works very well. In your tests, it could deliver the packages, but when you start looking into what is required - you might need to change your autopilot, triggering a rewrite of your software. By the way, we need a parachute now, and we won’t get as much flight time, so we need heavier batteries, but this will end up with us being in an even more controlled certification category… and it goes on.

Doing these rounds is undoubtedly fun, and you learn a lot, but, as a client, do you want to keep throwing money into the pit, instead of investing in a clear roadmap?


### Be Careful Using Upwork for High-Level Consultations

Upwork is quite a good place for finding projects. In exchange for a 10% fee, you get some discoverability and payment protection for hourly projects. The payment protection will only work though if you move your mouse and press keyboard keys enough during your work. I had this issue with one of my clients - having a very high-level consultation that did not require me to do any input on my computer, and it just so happened that the customer's credit card bounced.

Upwork then looked through my keyboard/mouse activity during the call and zeroed the activity periods without enough action (even though the app takes screenshots of your screen, and I could be seen on a call), removing a big chunk of my consultation time and never paying me for it. All of this is in line with their T&Cs. After the customer has their account revoked, you can manually add ‘lost time,’ but they can say no, and I don’t think there is much you can do about it, so use Upwork with caution


## Fin?

Over the past two years, I've worked increasingly with [DTE](https://dte.ai/), to the point where I'm well-embedded with their team.

<figure class="center">
    <iframe width="560" height="315" src="https://www.youtube-nocookie.com/embed/sl-AOec0p9w" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>
    <figcaption>DTE promo video
</figcaption>
</figure>


The company develops solutions (hardware and software) for the analysis of the element composition of alloys. I always explain how the system works by shooting lasers into alloys and using spectroscopy on the light emitted by the process to estimate the element content of alloys.

<figure class="center">
    <img src="/images/5_yr_consulting/image2.jpg" alt="A man on a volcano">
    <figcaption>Me, scaling a volcano in Iceland during one of my many visits to DTE’s headquarters. </figcaption>
</figure>

As a consultant, I've worked with about 50 companies now. There are only three that I would invest in, given a chance. DTE is one of those companies.

Even though I'm on a monthly retainer with one company at the moment, I still consult with others. But being so involved with a company day-to-day, you might be given a choice.

<figure class="center">
    <img src="/images/5_yr_consulting/image5.jpg" alt="Red pill blue pill on hands">
    <figcaption>Do you take the leap?</figcaption>
</figure>
