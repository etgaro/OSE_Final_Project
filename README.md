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
6. finally, you can run the code of choice (run.py for wall following, tree_run.py for tree row following) with the command "python + filename"
	make sure this is run from the turtlebot terminal!

- **note** we can read from the turtlebot on the PC but cannot send commands from PC to turtlebot.

----------------------------------------------------------------------------------------------------------

adding a state:
a state is added in the run file - NOT IN THE STATE MACHINE FILE
in the relevant file (run/tree_run) do the following steps:

1. go to "start_the_plan" function and create new state using the m.add_state function. this function needs 2 paramaters:
the state name, and the function it is connected to - referred to as a handler. 
   * note - m is the name of the state machine in our code 

2. once the state is created you need to define the handler. this function should have 2 parameters: self, system state. 
the second parameter is basically a connection to the current state of the system.
    * within the handler you write anything you want the robot to do while it is in this state.
    * the return command from the handler has three parameters:
        1. the name of the next state you want the robot to go to
        2. System_state (sending the information about the system state to the next state)
        3. a message to be printed when transitioning from current state to next state
    * of course, you can go from one state to a few different states using if/else commands depending on what you want to do.
    * at the beginning of the handler function you should have the following code to make sure that the robot is not shut down:
      #if CTRL+C is pressed - end state
      if rospy.is_shutdown():
          return "end_state", "finishhh"
