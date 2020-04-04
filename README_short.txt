~~~~~~~~~~~~ USI-THYMIO-SIMULATION ~~~~~~~~~~~~

Team GANYMEDE: 
Marco Ferri & Nadia Younis


------------- DEMO -------------

Demonstration videos of the Assignment can be found at the following links. 
Each Task has been done.
Any comment about them is available in the next sections.
- Compulsory Tasks: https://drive.google.com/open?id=1dUeoM8Tyf3Pm_AVI5jRvs_GjQvNQ4Gmg
- Bonus Tasks:      https://drive.google.com/open?id=1_h5_a9iirtNDcbzDH26-9EAjxXf5W89B


------- COMPULSORY TASKS -------

For executing Tasks 2 and 3, the following commands have to be run in the terminal:
- roslaunch usi_myt thymio_gazebo_bringup.launch world:=wall
- roslaunch usi_myt compulsory.launch

Robot's name is set automatically to the default value `thymio10`. 
Even though a second thymio (`thymio11`) will spawn, only the first one will be controlled.

Additional information about what is going on during the execution 
can be found in the terminal shown in the video: https://drive.google.com/open?id=1dUeoM8Tyf3Pm_AVI5jRvs_GjQvNQ4Gmg


If you want to execute task 1 instead, then these two commands are required:
- roslaunch usi_myt thymio_gazebo_bringup.launch world:=empty
- roslaunch usi_myt task1.launch
A specific video is also available for that task: https://drive.google.com/open?id=1UaJLM7M7bB0tV84vvJOh9_US36a3J_vd


--------- BONUS TASKS ----------

For executing Tasks 4 and 5, the following commands have to be run in the terminal:
- roslaunch usi_myt thymio_gazebo_bringup.launch world:=arena
- roslaunch usi_myt bonus.launch

Once again, robots' names are set to the default values, so you do not need to specify them manually.
The demo video (https://drive.google.com/open?id=1_h5_a9iirtNDcbzDH26-9EAjxXf5W89B) clearly shows 
how the robots are able to detect and avoid obstacles properly while randomly exploring the available space.

Please note that the `arena` world used in the video has been slightly modified: 
obstacles have been added or moved in order to create a convenient environment for demonstrating the robots' behavior more easily. 
The world found in the source code (which you will find when executing it) is instead the original that has been provided us.

