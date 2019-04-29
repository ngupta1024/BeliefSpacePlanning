bsp-ilqg
=============

Belief Space Motion Planning Using iLQG. Builds on top of iLQG Matlab implementation by Yuval Tassa and
the paper "Motion Planning under Uncertainty using Iterative Local Optimization in Belief Space", Van den berg et al., 
International Journal of Robotics Research, 2012

How To
=============

There are demo files provided in the main directory that you can straight away run and see planning scenarios.
Inside each of these demo files you can change the map to be loaded (i.e., the planning scenario) and you can change
the start and goal state.

A modular appraoch has been taken to implement this code, all motion models for robot and observation models for
sensing are based on base classes for each, enabling you to write your own models. Simple collision checking for
a circular (disk) like robot is provided along with a bunch of 2D maps. To implement your own planning scenario, write your 
own motion model and observation model. Then copy one of the demo files and in the part of code where the models are initialized 
replace with your own models.
