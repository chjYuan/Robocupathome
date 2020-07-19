robocup@home tutorial4
Yansong Wu & Helin Cao

--------------------------------------------------------------------

information for each exercise:
Exercise 1
  Robot: Tiago
  Packages:from2Dto3D 
  Solved by:Yansong Wu & Helin Cao
Exercise 2 
  Robot: Tiago
  Packages: plane_segmentation
  Solved by:Yansong Wu 
Exercise 3 
  Robot: Tiago
  Packages: object_recognition
  Solved by:Helin Cao & Yansong Wu 

-------------------------------------------------------------------- 

run step:
first of all, please download the bag (tiago_recording.bag) and put it the in the following path:plane_segmentation/bag/tiago_recording.bag
Compilation:
$ catkin_make
Execution:


for Exercise 2
$ roslaunch plane_segmentation seg.launch
  After launching this file, you can see a point cloud of the object on the table in rviz(you can also display table plane by setting the properties in the left menu). 


for Exercise3 and Exercise1
$ roslaunch object_detection recognition.launch
$ rostopic echo /segmentation/point3D
  After launching this file, you can see a YOLO V3 window. In the window, the object can be recognized and displayed with a label. To detect the exercise1, $ rostopic echo is used to show the 3d coordinate of object generated in the 2Dto3D process. 


note:
The $(find from2Dto3D)/config/objectname.yaml is a doucument, which can set the object you want to detect. The default object is "bottle". If you want to detect another object, you can just write the neme of the object in the document, such as "apple".

-------------------------------------------------------------------- 

file content:
  -/from2Dto3D
    -CMakeLists.txt
    -package.xml
    -config
     -objectname.yaml
    -/include/from2Dto3D
     -from2Dto3D.h
    -/src
     -from2Dto3D_node.cpp
     -from2Dto3D.cpp
  -/object_detection
    -CMakeLists.txt
    -package.xml
    -/include/object_detection
     -ObjectRecognition.h
    -/launch
      -recognition.launch     
    -/src
      -ObjectRecognition.cpp     
      -ObjectRecognition_node.cpp  
  -/perception_msgs
    -CMakeLists.txt
    -package.xml
    -/msg
      -PointCloudArray.msg
      -Rect.msg
      -RectArray.msg
    -/srv  
  -/plane_segmentation
    -CMakeLists.txt
    -package.xml
    -/bag
     -tiago_recording.bag
    -/include/plane_segmentation
      -PlaneSegmentation.h       
    -/launch
      -seg.launch        
    -/rviz
     -seg.rviz
    -/src
      -PlaneSegmentation.cpp 
      -PlaneSegmentation_node.cpp
--------------------------------------------------------------------  


