---
layout: post
title: Replacing my command prompt with emoji
author: Thomas Weng
comments: true
tags:
---

The title says it all:
<div class="cntr">
  <img src="../assets/19-11-15_1.png" />
  <div class="caption">
    My new emojified command prompt
  </div>
</div>

Why did I do this? Wanted to see if it was possible. And seeing emoji somehow brings me a lot of joy.

I did this on my Mac with [zsh](https://www.zsh.org/) and [Oh My Zsh](https://ohmyz.sh/). Here are the steps if you want to do the same in your own terminal.

# 1. In your `.zshrc`, add the emoji plugin to the list of plugins. 
```
plugins=(
  git alias-tips autojump emoji
)
```

# 2. In your zsh theme, update the PROMPT variable. 

You'll find the name of your current theme in your `.zshrc` file. The file for that theme is under `/.oh-my-zsh/themes/`. 

In my theme file, I replaced `>` with `$(random_emoji animals)`, which loads a random animal emoji as my terminal prompt each time I open a new terminal. Other options are available in the emoji plugin [documentation](https://github.com/robbyrussell/oh-my-zsh/tree/master/plugins/emoji).

```
# Old prompt
PROMPT='
${_current_dir}$(git_prompt_info) %{$fg[$CARETCOLOR]%}>%{$resetcolor%} '
# New prompt
PROMPT='
${_current_dir}$(git_prompt_info) %{$fg[$CARETCOLOR]%}$(random_emoji animals)%{$resetcolor%} '
```

To see changes, reload zsh with `source ~/.zshrc` or open and close the terminal.

# 3. Change the command prompt emoji with the `random_emoji` command

Step 2 replaces your prompt with a random emoji for each new terminal. If you get tired of the current emoji, you can change it with the command `random_emoji`. It outputs a random emoji and also randomly changes the emoji of your prompt.

<div class="cntr">
  <img src="../assets/19-11-15_2.png" />
  <div class="caption">
    Change the command prompt emoji with `random_emoji`
  </div>
</div>

Enjoy!