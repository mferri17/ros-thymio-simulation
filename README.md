# USI-THYMIO-SIMULATION

Università della Svizzera Italiana

Robotics 2020 | **Assignment 2**

Authors: **Marco Ferri** & **Nadia Younis**


## Demos

Demonstrational videos of the Assignment can be found as follows.
- Task 1:
- Tasks 2 and 3:
- Tasks 4 and 5: 


## Getting started

### Task 1

For executing the first task, the following commands have to be run in the terminal:

```
roslaunch usi_myt thymio_gazebo_bringup.launch name:=thymio10 world:=empty
roslaunch usi_myt task_1.launch robot_name:=thymio10
```

### Tasks 2 and 3

For executing these tasks, the following commands have to be run in the terminal:

```
roslaunch usi_myt thymio_gazebo_bringup.launch name:=thymio10 world:=wall
roslaunch usi_myt task_3.launch robot_name:=thymio10
```

In case the simulation has to be runned multiple times, than also the command `rosservice call /gazebo/reset_simulation` is needed for resetting the Gazebo simulation enviroinment, returning the Thymio to the original position in order to be faced against the wall.

### Tasks 4 and 5

