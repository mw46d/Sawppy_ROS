# A ROS Package Collection for Sawppy the Rover

## Intro

This collection is heavily influenced by the ROS_Adruino_Bridge packages and by the original
SGVHAK_Rover sources for all the calculations.

## ROS Setup

This directory is supposed to be called `mw` in your catkin source directory. I keep different trees
which all live in `mw` but have otherwisde nothing in common. That's the reason for the more
descriptive repository name.

So far, it's just Python code, so it should build cleanly with `catkin_make`.

There are launch files in here, which work for my setup. Take them with a grain of salt;-)

I'm currently running the of *Kinetic* on an Up-Board, but I don't believe, there should be many
problems with any newer ROS-1 version. No idea about ROS-2:-(
