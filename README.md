# USI-THYMIO-SIMULATION

Università della Svizzera Italiana

Robotics 2020 | **Assignment 2**

Team Ganymede: **Marco Ferri** & **Nadia Younis**


## Demos

Demonstration videos of the Assignment can be found at the following links. Any comment about them is available in the next sections.
- Compulsory Tasks: https://drive.google.com/open?id=1dUeoM8Tyf3Pm_AVI5jRvs_GjQvNQ4Gmg
- Bonus Tasks:      https://drive.google.com/open?id=1_h5_a9iirtNDcbzDH26-9EAjxXf5W89B



## Getting started

All the following tasks have to be run into an Ubuntu machine with a working ROS + Gazebo environment. The ROS package has to be built with `catkin` in order to be executed.


### Compulsory Tasks

For executing Tasks 2 and 3, the following commands have to be run in the terminal:

```
roslaunch usi_myt thymio_gazebo_bringup.launch world:=wall
roslaunch usi_myt compulsory.launch
```

Robot's name is set automatically to the default value `thymio10`. Even though a second thymio (`thymio11`) will spawn due to the first command, the implemented controller will only move the first robot (the one at `0,0`), ignoring the other.

In case the simulation has to be run multiple times, then also the command `rosservice call /gazebo/reset_simulation` is needed for resetting the Gazebo simulation environment, returning the Thymio to the original position in order to be faced directly against the wall. The original `wall` world provided has been modified to put the wall nearer to the spawn position and to be placed diagonally with respect to the x-axis.

Additional information about what is going on during the execution can be found in the terminal shown in the [demo video](https://drive.google.com/open?id=1dUeoM8Tyf3Pm_AVI5jRvs_GjQvNQ4Gmg).


### Bonus Tasks

For executing Tasks 4 and 5, the following commands have to be run in the terminal:

```
roslaunch usi_myt thymio_gazebo_bringup.launch world:=arena
roslaunch usi_myt bonus.launch
```

Once again, robots' names are set to the default values, so you do not need to specify them manually.

The [demo video](https://drive.google.com/open?id=1_h5_a9iirtNDcbzDH26-9EAjxXf5W89B) clearly shows how the robots are able to detect and avoid obstacles properly while randomly exploring the available space. If an obstacle is exactly in front of the Thymio, it turns completely by 180 degrees and then keeps moving. Instead, if the obstacle is on the side (mainly detected by one left/right sensor only), then the robot just turns a little bit in the proper direction. Furthermore, when a Thymio encounters another one, it handles it as a normal obstacle; both the two robots will stop, turn around and restart moving in such a way that would not make them crash against each other.

Please note that **the `arena` world used in the video has been slightly modified**: obstacles have been added or moved in order to create a convenient environment for demonstrating the robots' behavior more easily. The world found in the source code (which you will find when executing it) is instead the original that has been provided us.

