# OSE_Final_Project
**Autonomous navigation for agriculture robot**

##Relevant files for navigation in line of trees:
###-tree_run.py
###-scan.py
###-State_machine.py

##Other files:
-run.py - for keeping distance from wall\
-others - testing and understanding the environment and code

##How to start the robot:

1. make sure the turtlebot and PC are using the same wifi
2. open terminal and get updated ip address (ifconfig)
3. in the terminal, connect to the turtlebot using the following command: ssh turtlebot@192.168.###.236, password: a
	- if we want to transfer graphic applications as well we need to use command ssh -X turtlebot@192.168.###.236
4. update the ip addresses in the bashrc files that are accessed in one of three ways:
	- command: gedit ~/.bashrc (opens the txt file)
	- command: nano ~/.bashrc (opens the file to edit in terminal)
	- in the home files - select under 'view' the 'show hidden files' option and open the .bashrc file like in the first option
   what needs to be updated?
	- in the turtlebot - the ROS_MASTER_URI is the turtlebot IP, and ROS_IP is the turtlebot IP
	- in the the PC - the ROS_MASTER_URI is the turtlebot IP, and ROS_IP is the PC IP
5. in the turtlebot terminal run the following command: roslaunch turtlebot3_bringup turtlebot3_robot.launch this will start the roscore.

- now its time to send python script to the robot - but needs to be run from the turtlebot terminal!!!!
- **note** we can read from the turtlebot on the PC but cannot send commands from PC to turtlebot.
- rqt - usefull tool, can have control of rostopic and easy way to use teleop as well as many other things.
- rviz - tool for visualizing the info from the sensors - Lidar and camera. could (and should) be open on PC since it is only reading.
- connect to turtle files from PC using network locations or something like that.

###How to start the simulation:

1. open gazebo: ~/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch$ roslaunch turtlebot3_world.launch
2. open rviz: ~/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch$ roslaunch turtlebot3_gazebo_rviz.launch
3. teleop: ~/catkin_ws/src/turtlebot3/turtlebot3_teleop/launch$ roslaunch turtlebot3_teleop_key.launch