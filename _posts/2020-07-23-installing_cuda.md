---
layout: post
title: Installing NVIDIA drivers and CUDA on Ubuntu 16.04 and 18.04
author: Thomas Weng
comments: true
tags: 
- how-to
---

There are several ways to install NVIDIA drivers. This method using a runfile is the one that works most reliably for me. 

1. Determine the driver version and CUDA version you need. The versions will depend on the GPU you have and the code you want to run. For example, specific pytorch and tensorflow versions are tied to specific CUDA versions. 

2. Download the driver and CUDA version as a runfile from the NVIDIA website: [CUDA Downloads](https://developer.nvidia.com/cuda-downloads)

3. If you already have nvidia installed, uninstall it: `sudo apt-get purge nvidia-*` and reboot your machine. 

4. Upon restart, drop into the tty terminal with `Ctrl+Alt+F1` and log in. 

5. Stop running the gui. On Ubuntu 16.04 run `sudo service lightdm stop`, on 18.04 run `sudo systemctl stop gdm3.service`.

6. Find the runfile you downloaded from step 2 and run it: `sudo sh ./NAME_OF_RUNFILE.run`

7. Follow the prompts to install the driver and CUDA. If the installation fails, you will need to check `/var/log/nvidia-installer.log` or `/var/log/cuda-installer.log` to find out why. You can test if the installation was successful by running `nvidia-smi`.Check the reported driver and CUDA versions. 

8. Restart the gui. On Ubuntu 16.04 run `sudo service lightdm start`, on 18.04 run `sudo systemctl start gdm3.service`.

9. Leave the tty terminal and return to the gui: `Ctrl+Alt+F7`.

For more detailed instructions, see the [NVIDIA CUDA Installation Guide](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html).