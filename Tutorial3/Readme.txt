/******************Execise1*************************/
You can find the required file "map.pgm"and "map.yaml" in subfolder "small_office" and "tutorial_office" respectively.
/******************Execise2*************************/
1. Make sure that the "Tiago" package has already installed in you workspace
2. Add the given package "robot_navigation" into your workspace and then compile the package with 
   $ catkin_make
3. In your workspace run the following command:
   $ source devel/setup.bash 
   $ roslaunch robot_navigation navigation.launch
When all things start, you will see tiago move between the points A,B,C.
/******************Execise3*************************/
1. Add the given package "robot_nav_motion" into your workspace and then compile the package with 
   $ catkin_make
2. In your workspace run the following command:
   $ source devel/setup.bash
   $ roslaunch robot_nav_motion navigation.launch
When all things start, you will see tiago move between the points A,B,C and lift the table at the point C for one time and then continue patroling like the previous task.  
 
