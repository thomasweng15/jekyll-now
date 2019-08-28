---
layout: post
title: Terminal tips
author: Thomas Weng
comments: true
tags:
- programming
- productivity
---

The terminal is an essential tool[^1], but also one whose tasks are most easily automated and optimized.

Most of the time spent on the command line is on performing non-value-added tasks, like moving files around, executing programs, installing dependencies, and checking system status.[^2]
These tasks are necessary, but don't ultimately end up in your final product.

Therefore, you should aim to spend as little time in the terminal as possible, focusing instead on value-added tasks like writing programs, analyzing data, making visualizations, etc.

Here are some ways to automate or speed up terminal tasks. I'm assuming you use bash, but the items here are applicable to most terminals: 
1. Turn series of commands into scripts
2. Avoid typing out commands
3. Check system status using htop
4. Split panes with tmux
5. Download files faster using aria2
6. Use key-based authentication for ssh and Github
7. Try different terminals

# 1. Turn series of commands into scripts

This one seems obvious, but if you run the same series of commands often (e.g. building and running code), turn it into a script.

# 2. Avoid typing out commands

Avoid typing out commands, especially if you have typed a similar one already. Tab-autocomplete aggressively, use `Ctrl+r` to [reverse-i-search](https://lifehacker.com/ctrl-r-to-search-and-other-terminal-history-tricks-278888) your command history, and keep commands that are relevant for a project in a README.

# 3. Check system status using htop

Check system status using [htop](https://hisham.hm/htop/), an improved version of the top command. 

![](../assets/19-08-27_1.png)

Besides checking system usage, I also use this to find (`F4`) and kill (`F9`) processes as root.

# 4. Split panes with tmux

Use [tmux](https://github.com/tmux/tmux/wiki) to create and switch between multiple terminal instances. Another benefit of using tmux is that terminals will persist even if your ssh connection drops.



# 5. Download files faster using aria2

Use [aria2](https://aria2.github.io/)

`aria2c -x8 URL`

`-x8` specifies how many connections to open to parallelize the download.

# 6. Use key-based authentication for ssh and Github

Use key-based authentication for ssh and Github

# 7. Try different terminals

You can also explore beyond bash and try terminals with more features. I use [zsh](https://ohmyz.sh/) as my default shell with plugins for jumping between directories, better autocomplete, etc.

--- 

_Many thanks to [Rosario Scalise](https://personalrobotics.cs.washington.edu/people/) for introducing me to tmux! Shout out to Cherie Ho for her [post on upgrading the MacOS terminal](https://www.cherieho.com/2018-11-24-upgrade-terminal/) with `zsh`. Thanks to Abhijat for introducing me to aria2!_

Footnotes
 
[^1]: If you're a beginner with the terminal, [this reference](https://swcarpentry.github.io/shell-novice/reference/) from Software Carpentry is a good starting point. 
[^2]: Note that I consider working in a terminal-based editor like vim different from working in the terminal itself.