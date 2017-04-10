# r4a_MCL

*Code by Aspa Karanasiou and Chrysa Gouniotou*

This code implements the MCL algorithm (Monte Carlo Localization). Before you execute the code, you should educate yourself with the basics of PFs (Particle Filters) and MCL from the following links:
- MCL: (https://en.wikipedia.org/wiki/Monte_Carlo_localization)[https://en.wikipedia.org/wiki/Monte_Carlo_localization]
- (Slides from Burgard, Stachniss about PFs and MCL)[http://ais.informatik.uni-freiburg.de/teaching/ss11/robotics/slides/11-pf-mcl.ppt.pdf]
- Thrun's online class:
  - https://www.youtube.com/watch?v=H0G1yslM5rc 
  - https://www.youtube.com/watch?v=qQQYkvS5CzU
  - https://www.youtube.com/watch?v=lwg_KI3UewY 

The first step is to clone the following repository in your catkin workspace and build the code.

```
git clone https://github.com/robotics-4-all/r4a_MCL
```

In order to execute the code you must already a robot or  a simulator (e.g. [STDR Simulator](https://github.com/stdr-simulator-ros-pkg/stdr_simulator)). You can start the STDR simulation by executing:

```
roslaunch art_localization_particle_filters_ogm robocup_map_plus_robot_with_laser.launch 
```
