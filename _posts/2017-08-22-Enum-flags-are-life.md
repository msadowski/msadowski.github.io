---
layout: post
title: Enum flags in C# are life
description: "Enum flags can make your life much easier"
modified: 2017-08-22
comments: true
tags: [Programming, C#]
image:
  feature: flags.jpg
  credit: Pixabay.com
---
For the 5 years I've been working on and off with C# I've never encountered enum flags. Until yesterday. I thought the concept would make a great blog post so here it is!

<!-- more -->

# Yesterday's approach to enums

If you asked me to hold multiple enums in a variable here is how I would've done it before my enlightenment:

{% highlight csharp %}
using System;
using System.Collections.Generic;

public class HelloWorld
{
    static public void Main ()
    {
        Console.WriteLine("Hello Mono World");
        List<Fruits> basket = new List<Fruits>()
        {
          Fruits.Apples,
          Fruits.Watermelons,
          Fruits.Pears
        };
        Console.Write("In our basket we have: ");
        foreach (Fruits fruits in basket)
        {
          Console.Write(fruits + " ");
        }
    }
}

public enum Fruits
{
  Apples,
  Bannanas,
  Watermelons,
  Raspberries,
  Pears
}
{% endhighlight %}

# The new ways

Since we are learning every day here is how we could perform something similar with *enum flags*:

{% highlight csharp %}
using System;

public class HelloWorld
{
    static public void Main ()
    {
        Console.WriteLine("Hello Mono World");
        Fruits basket = Fruits.Apples | Fruits.Watermelons;
        Console.WriteLine($"In our basket we have {basket}");
    }
}

[Flags]
public enum Fruits
{
  Apples = 1,
  Bannanas = 2,
  Watermelons = 4,
  Raspberries = 8,
  Pears = 16
}

{% endhighlight %}

The nice thing about flags is that C# will automatically print out the name of all the enums instead of the number. Note that the values of enums are all powers of 2, ensuring that the number stored in a variable (in our case _basket_) is unique.

I hope this post was quite informative for you. Please remember to ABC (Always Be Coding)!
