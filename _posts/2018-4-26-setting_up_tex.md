---
layout: post
title: Setting up TeX math rendering for your blog or website
author: Thomas Weng
comments: true
tags:
- blogging
---

Setting up \\(\TeX\\)math typesetting on your website is one of those tasks that can seem really difficult if you don't know what to look for. Here's a simple way to do it. 

I used Khan Academy's [KaTeX](https://khan.github.io/KaTeX/) typesetting library for my blog. KaTeX is fast, self-contained, and works largely as you would expect. Here are the instructions:

1. Include KaTeX on your site by adding the [required reference tags and scripts](https://github.com/Khan/KaTeX/blob/master/contrib/auto-render/README.md) to your html template[1]. See 
[this commit](https://github.com/thomasweng15/thomasweng15.github.io/commit/a8ae4f214dd8bec31e29c62f3cdc79d2e9b761f8) for the changes I made to get it working on this blog.

2. Typeset math by enclosing your notation within <code class="text">\\(</code> and <code class="text">\\)</code> for inline typesetting, or <code class="text">\\[</code> and <code class="text">\\]</code> for typesetting on a separate line.

    For example, with inline typesetting, <code class="text">\\(\alpha\\)</code>, becomes \\(\alpha\\). With non-inline typesetting, <code class="text">\\[x^2 + y^2 + z^2 = r^2\\]</code> becomes: \\[x^2 + y^2 + z^2 = r^2\\]

3. If you have any rendering issues, check your console for error messages. Also see take a look at the full documentation on [Github](https://github.com/Khan/KaTeX).

---
[1] You can also download these files and host them on your server directly instead of getting them from the cdn.