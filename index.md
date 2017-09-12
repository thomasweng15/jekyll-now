---
layout: default
title: Hi, I'm Thomas.
---

<h2>Hi, I'm Thomas.</h2>

I'm a software engineer at Microsoft. I've worked on the Bing Answers & Widget Experiences team for almost two years, contributing to projects such as the [Bing 2016 Election Answers](http://blogs.bing.com/search?tagname=election%202016&groupid=10), the [Flight Booking Answer](https://www.bing.com/search?q=book%20flight), and Bing at Work.

Download a copy of my resume here: [resume.pdf](../resources/resume.pdf){:target="\_blank"}

Before Microsoft, I double-majored in Computer Science and Economics at Yale University. My senior thesis on real-time robot perception of human nonverbal behavior won the CS departmental prize in research. My lab and I went on to publish in [HRI 2016](http://scazlab.yale.edu/sites/default/files/files/ADMONI_hri16.pdf) and [ICRA 2016](http://hennyadmoni.com/documents/admoni2016icra.pdf).

Watch the demo video from my thesis submission below:

<iframe class="ytv" src="https://www.youtube.com/embed/v9yymuaYLtw" frameborder="0" allowfullscreen></iframe>

### What's on this blog?

I'm still breathing life into the site, but I plan to I post writeups of the projects I'm working on, along with short essays on various technical topics. Stay tuned!

<div class="posts">
  <h3 class="posts_title">Recent Posts</h3>
  <ul class="posts">
    {% for post in site.posts %}
      <li><span>{{ post.date | date_to_string }}</span> &raquo; <a href="{{ post.url }}">{{ post.title }}</a></li>
    {% endfor %}
  </ul>
</div>

### How can I reach you?

Email is the best way to reach me: __thomas.weng11@gmail.com__
