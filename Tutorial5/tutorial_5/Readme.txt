Robot:Tiago
Team member:Helin Cao
            Yansong Wu
            Cong Wang
            Chengjie Yuan
1.Fisrt add all the packages in the tiago workspace and then compile all of them, and then source the bash file.
  $ source devel/setup.bash
2.Open an terminal and enter in the tiago mode using
  $ tiago_mode
3.Change the map to the "pal_room_22" using:
  $ rosservice call /pal_map_manager/change_map "input: 'pal_room_22'"
3.Then launch the node in the following order
  $ roslaunch object_detection recognition.launch 
4.Openning another terminal then launch 
  $ roslaunch from2Dto3D transform.launch 
5.In a new terminal launch
  $ roslaunch robot_nav_motion rviz.launch 
6.In a new terminal launch
  $ roslaunch robot_nav_motion test_whole.launch 
  
