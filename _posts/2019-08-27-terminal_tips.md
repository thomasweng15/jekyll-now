---
layout: post
title: Terminal tips
author: Thomas Weng
comments: true
tags:
- programming
- productivity
---

The terminal is an essential tool,[^1] but also one whose tasks are most easily automated and optimized.

Most of the time spent on the command line is on non-value-adding tasks, like moving files around, executing programs, installing dependencies, and checking system status.[^2]
These tasks are necessary, but do not end up in your final deliverable, i.e. a publication, code, or other project output.

Therefore, you should aim to spend as little time in the terminal as possible, focusing instead on value-adding tasks like writing programs, analyzing data, making visualizations, etc.

Here are some ways to automate or speed up terminal tasks. I'm assuming you use bash, but the items here are applicable to most terminals. 
1. [Shorten commands using aliases and scripts](#1-shorten-commands-using-aliases-and-scripts)
2. [Use reverse-i-search to find past commands](#2-use-reverse-i-search-to-find-past-commands)
3. [Check system status using htop](#3-check-system-status-using-htop)
4. [Split one terminal into several with tmux](#4-split-one-terminal-into-multiple-with-tmux)
5. [Download files faster using aria2](#5-download-files-faster-using-aria2)
6. [Set up key-based authentication for ssh and Github](#6-set-up-key-based-authentication-for-ssh-and-github)
7. [Try out other terminals](#7-try-out-other-terminals)

# 1. Shorten commands using aliases and scripts

This one seems obvious, but if you run the same set of commands often, turn them into [aliases](https://www.digitalocean.com/community/tutorials/an-introduction-to-useful-bash-aliases-and-functions). 
I use aliases for computers that I ssh into often, e.g. 

```shell
#!/bin/bash
alias hostname="ssh <username>@<hostname>"
```

Another use case is for aliases is to shorten series of commands, like navigating to a directory and running a script.

```shell
#!/bin/bash
alias intera="cd ~/catkin_ws && source intera.sh"
```

If a series of commands is longer or more complicated (e.g. building and running code), it may be better to turn it into a [shell script](https://www.digitalocean.com/community/tutorial_series/an-introduction-to-shell-scripting), which will allow you to take advantage of for loops, command line arguments, etc. 

# 2. Use reverse-i-search to find past commands

Related to the first point, avoid typing out commands, especially if you have typed a similar one already. Besides using tab-autocompleting aggressively, I also use [reverse-i-search](https://lifehacker.com/ctrl-r-to-search-and-other-terminal-history-tricks-278888) all the time to search my command history. Activate reverse-i-search using `Ctrl+r` and then type in a query to find matches. Hit `Ctrl+r` again to find the next match.

<div class="cntr">
  <img src="https://media.giphy.com/media/lOsUAIXzpcK3nr6Tej/giphy.gif" />
  <div class="caption">
  Demonstrating reverse-i-search.
  </div>
</div>

# 3. Check system status using htop

[htop](https://hisham.hm/htop/) is an improved version of the `top` command, with colors, better navigation, and other features. 

<div class="cntr">
  <img src="../assets/19-08-27_1.png" />
  <div class="caption">
  Screenshot of htop.
  </div>
</div>

Besides checking system usage, I also use this to find (`F4`) and kill (`F9`) processes as root (`sudo htop`).

# 4. Split one terminal into several with tmux

Use [tmux](https://github.com/tmux/tmux/wiki) to create and switch between multiple terminal panes within one window. 

<div class="cntr">
  <img src="https://media.giphy.com/media/lquszCDcatgZlrC06g/giphy.gif" />
  <div class="caption">
  Demonstrating how to split panes with tmux. Note that my keybindings are different from the tmux defaults.
  </div>
</div>

Another benefit to tmux is that these terminals will persist even if your ssh connection drops. Simply run `tmux attach` after ssh-ing back in to return to your panes.

# 5. Download files faster using aria2

[aria2](https://aria2.github.io/) is an alternative to `wget` or `curl` that parallelizes downloads:

```console
$ aria2c -x8 <URL>
```

`-xN` specifies how many connections to open for parallelization.

# 6. Set up key-based authentication for ssh and Github

Instead of typing your password each time you ssh or push/pull from Github, use key-based authentication. It only takes a few minutes and has the dual benefit of being easier and more secure than password authentication.

To set up key-based authentication for ssh, see this DigitalOcean post: [How To Set Up SSH Keys](https://www.digitalocean.com/community/tutorials/how-to-set-up-ssh-keys--2). You'll need to do this once per computer you want to `ssh` into. For Github, follow the steps outlined here: [Connecting to Github with SSH](https://help.github.com/en/articles/connecting-to-github-with-ssh)

# 7. Try out other terminals

You can also explore beyond bash and try terminals with more features. I use zsh as my default shell with plugins for jumping between directories, better autocomplete, etc. If you're interested in zsh, check out these articles on how to get started: [Articles from the Oh My Zsh wiki](https://github.com/robbyrussell/oh-my-zsh/wiki/Articles).

--- 

_Many thanks to [Cherie Ho](http://www.andrew.cmu.edu/user/abhijatb/) for editing this post and introducing me to zsh, [Leah Perlmutter](https://homes.cs.washington.edu/~lrperlmu/) for introducing me to reverse-i-search, [Rosario Scalise](https://personalrobotics.cs.washington.edu/people/) for tmux, and [Abhijat Biswas](http://www.andrew.cmu.edu/user/abhijatb/) for aria2!_

Footnotes
 
[^1]: If you're a beginner with the terminal, [this reference](https://swcarpentry.github.io/shell-novice/reference/) from Software Carpentry is a good starting point. 
[^2]: I consider working in a terminal-based editor like vim different from being on the command line itself.