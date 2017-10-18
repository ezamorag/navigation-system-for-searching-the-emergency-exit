# navigation-system-for-searching-the-emergency-exit
I developed a complete navigation system for a differential mobile robot (iRobot). This navigation system is able to find the emergency exit in the building. A demo is shown in the following video 

https://www.youtube.com/watch?v=RAj70AGsXls

The video shows some examples of these emergency signs and the robot searching for the exit. It uses a RGBD camera (Kinect) to detect emergency signs and determine the direction that it should move. The same sensor provides information on obstacles such as chairs, walls, tables, people, etc. It uses a SLAM method (Gmapping method) to build a 2-D map as it moves; at the beginning the robot does not know the environment, i.e. it does have a map. It uses a planning method (A* search) to find the shortest path. This method was modified to integrate the information of emergency signs. The planned path is executed by a P controller and a method of obstacle avoidance (Smooth Nearness Diagram) was added. All software is written in Python and C/C++ on ROS platform (Robot Operating System). 

The most interesting part of this project was to implement the system of perception which must detect, recognize and interpret the emergency signs to know how to move. Furthermore, it was necessary to implement a system to prevent repeated detections which was coupled to the SLAM method.  

