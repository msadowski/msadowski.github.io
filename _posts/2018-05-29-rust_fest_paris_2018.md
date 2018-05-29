---
layout: post
title: Thoughts on Rust Fest Paris 2018
description: "I've been super lucky to attend Ruste Fest in Paris this year. In this post I try to summarize the experience"
modified: 2018-05-29
comments: true
tags: [Programming, Rust]
image:
  feature: rustStickers.jpg
---

Since I've been trying to learn Rust for over a year now I thought that participating in Rust Fest would be a great boost in both motivation to keep digging into Rust and in learning new things and being inspired. It turns out I was not wrong!

<!-- more -->

## Prequel - motivation

For me the main motivation for learning Rust is to be able to write code that is safe and can handle multithreading really well while helping me to avoid bugs and issues. Ideally out of the box! In the future I'm hoping to use Rust in Robotics project both in high level scenarios (ideally integrating it with Robot Operating System) or in low level embedded systems (for example hardware drivers for sensors or actuators).

## Day 1 - conference

There were 4 talks that I found particularly interesting given my motivation for attending Rust Fest. 

### Learning How to Learn with Vaidehi Joshi

Vaidehi is an author of [Base CS](https://medium.com/basecs) series. For a year Vaidehi was teaching herself computer science and documenting everything on her blog. I found her story to be very inspiring, especially for those of us without formal CS education. The other interesting things you can learn from this talk are facts about Richard Feynman and his approach to learning.

{::nomarkdown}
<iframe width="560" height="315" src="https://www.youtube.com/embed/23lRkdDXqY0?rel=0" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>
{:/nomarkdown}

### Building Reliable Infrastructure in Rust by Tyler Neely

Very interesting talk about infrastructure and testing in Rust. It made me want to look more into testing and test automation, especially in robotics context (although the talk does not mention robotics at all). I really liked the idea of Property Testing and will definitely look into it more.

{::nomarkdown}
<iframe width="560" height="315" src="https://www.youtube.com/embed/hMJEPWcSD8w?rel=0" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>
{:/nomarkdown}

### Vector Graphics Rendering on the GPU in Rust with lyon by Nicolas Silva

Really relevant talk given my [last post on svg graphics in C#]({{ site.url }}/WPF-vector-graphics-tutorial/) (although I never went anywhere near the depth of this talk). If you are interested in graphics handling in your applications then I very much recommend you watch this talk. It's very interesting and the vector graphics renderer used in the talk is silky smooth!

{::nomarkdown}
<iframe width="560" height="315" src="https://www.youtube.com/embed/2Ng5kpDirDI?rel=0" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>
{:/nomarkdown}

### Monotron: Making a 80s style computer with a $20 dev kit by Jonathan Pallant

How many hoops do you need to go through to simulate an old system with VGA graphics capabilities on a bare metal processor? It turns out there are plenty. Very engaged speaker, I highly recommend this talk to everyone, even if you are not that interested in embedded applications.

{::nomarkdown}
<iframe width="560" height="315" src="https://www.youtube.com/embed/pTEYqpcQ6lg?rel=0" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>
{:/nomarkdown}

### And some more

I only described the talks that I found particularly interesting for my applications and interests however the other speakers did awesome job as well! If you want to learn more about RUST in the context of web assembly, networking, c++ integration etc then you ought to check out [Rust Fest Paris playlist](https://www.youtube.com/watch?v=23lRkdDXqY0&list=PL85XCvVPmGQgdqz9kz6qH3SI_hp7Zb4s1).

## Day 2 - RustBridge workshop

<figure class="center">
  <img src="{{site.url}}/images/ferris.gif" alt="Ferris the Rust crab">
	<figcaption>Ferris the crab <a href="https://rustbridge.github.io/a-very-brief-intro-to-rust/#1" title="Ferris in Action">[Source]</a></figcaption>
</figure>

The second day of Rust Fest I attended RustBridge workshop in Mozilla headquarters in Paris. Even though I went half way through [the book](https://doc.rust-lang.org/book/second-edition/index.html) I decided to attend RustBridge anyway (one could say I wanted to Polish my rust (double pun intended)). Looking back it was a good choice, I learned about some very useful tools (_cargo check_, _cargo doc_, _rustup docs_) and resources ([Exercism](http://exercism.io/languages/rust/about), [into_rust](http://intorust.com/), [rustlings](https://github.com/rustlings/rustlings)).

During the workshop we were going through some of the Rust syntax, structure, data types and et the end we were creating an emergency compliment website.

Another thing I took from this workshop is how welcoming is the Rust community. Everyone was very helpful and welcoming. Does it mean that RustBridge workshop is coming to Geneva in the coming months? Quite probably!

If what you've read so far sounds interesting You can find the workshop slides [here](https://rustbridge.github.io/a-very-brief-intro-to-rust/#1).

## Day 3 - Impl days with Embedded Working Group

My last day of Rust Fest Paris 2018 I joined the embedded working group for couple of hours of hacking and learning to use embedded Rust on a STM32F3DISCOVERY board. Throughout the day I was working on the [Discovery book](https://japaric.github.io/discovery/README.html) by [Jorge Aparicio](https://github.com/japaric).

If you are looking into learning embedded Rust I highly recommend you take a look into this resource. What I particularly liked about this set of tutorials so far is that you start by working directly on registers and afterwards you see how easy things can become once you start looking into [svd2rust crate](https://docs.rs/svd2rust/0.13.1/svd2rust/).

Also making LEDs blink is very satisfying
<figure class="center">
  <img src="{{site.url}}/images/disco_led.gif" alt="STM32F3Discovery disco lights">
	<figcaption>This is awesome <a href="https://japaric.github.io/discovery/README.html" title="Embedded Rust Discovery">[Source]</a></figcaption>
</figure>

## Summary

I learned a lot, the investment in this conference was very well worth it. Not only I learned a lot about Rust both as a language for embedded platform but also got to experience the welcoming community. I'm looking forward to joining the next Rust Fest, perhaps as a speaker. 

### Paris night life recommendation

Paris has an [Otaku Social Club](http://www.otakusocialclub.com/) where you can play Mario Kart with your friends. If you are lucky you can also learn a bit about fighting with **REAL** lightsabers!

<figure class="center">
  <img src="{{site.url}}/images/lightsabers.jpg" alt="Fighting jedi style">
</figure>