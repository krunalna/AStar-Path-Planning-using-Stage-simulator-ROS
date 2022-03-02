# A-Star-Path-Planning-using-Stage-simulator-ROS


Autonomously plan and execute a path for a robot in the Stage simulator from a start location to a goal location, given a map.

Steps to follow:

    Create package inside /catkin_ws/src/

catkin_create_pkg astar std_msgs geometry_msgs rospy roscpp

    Copy files inside package and follow:

cd ~/catkin_ws
catkin_make
source ~/.bashrc

    Grant execution permission to the script or .py file

chmod +x <.py file directory>

    Run program by roslaunch:

roslaunch astar astar.launch
