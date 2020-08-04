---
layout: post
title: Compressing Videos with ffmpeg
author: Thomas Weng
comments: true
tags: 
- how-to
---

Publication venues in robotics often allow supplementary video submissions. There are often strict size limits on video submissions. To meet these limits, it's often necessary to compress the video. 

I used to compress my videos using [Compressor](https://www.apple.com/final-cut-pro/compressor/) after making my videos with iMovie or Final Cut Pro. But Compressor was slow and buggy at times - the compression process would hang and only work after restarting, the compressed video was just a blank screen, etc. I'm sure Compressor is a powerful tool for a video editing expert, but I'm just a researcher with a simple task in mind. 

Now I just use [ffmpeg](https://ffmpeg.org/) to compress my videos. I share/export the video from Final Cut Pro and then use this command to compress it:

`ffmpeg -i VIDEO.mov -vcodec libx264 -crf 20 VIDEO_COMPRESSED.mov`

The `-crf` flag specifies the compression factor and it ranges from 0 (lossless) to 51 (worst quality), with 23 as the default. If I'm trying to get under a size limit, I'll run the command multiple times, playing with this value until I meet the requirement.

ffmpeg can also be used for converting between video formats, trimming, cropping, etc. Hope this helps!