from State_Machine import StateMachine
from scan import scanner
from random import random
import time
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan



class RunRobot:
    def __init__(self):
        # initialize

        # tell user how to stop TurtleBot
        rospy.loginfo("To stop TurtleBot CTRL + C")
        rospy.on_shutdown(self.terminate)

        # Create a publisher which can "talk" to TurtleBot and tell it to move
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.scanner = scanner()

        self.move_cmd_straight = Twist()
        self.move_cmd_straight.linear.x = 0.3
        self.move_cmd_straight.angular.z = 0.0

        self.move_cmd_right = Twist()
        self.move_cmd_right.linear.x = 0
        self.move_cmd_right.angular.z = -0.4

        self.move_cmd_left = Twist()
        self.move_cmd_left.linear.x = 0
        self.move_cmd_left.angular.z = 0.4

        self.r = rospy.Rate(10)

        self.keep_from_wall_max = 0.30
        self.keep_from_wall_min = 0.20

        self.cmd_vel.publish(self.move_cmd_straight)
        # wait for 0.1 seconds (10 HZ) and publish again


    def move_forward_handler(self, System_state):

        # publish command and wait for 0.1 seconds (10 HZ)
        if(self.is_parallel()):

            self.cmd_vel.publish(self.move_cmd_straight)
            rospy.sleep(0.5)

            newState, transition = self.adapt_distance()

            return newState, System_state, transition

        else:

            newState, transition = self.adapt_angle()

            return newState, System_state, transition

    def correct_clockwise(self,System_state):

        self.cmd_vel.publish(self.move_cmd_right)
        self.r.sleep()

        self.cmd_vel.publish(Twist())

        return "move_forward", System_state, "from clockwise to forward"


    def correct_un_clockwise(self, System_state):

        self.cmd_vel.publish(self.move_cmd_left)
        self.r.sleep()

        self.cmd_vel.publish(Twist())

        return "move_forward", System_state, "from un_clockwise to forward"

    def correctright_handler(self,System_state):

        self.cmd_vel.publish(self.move_cmd_right)
        rospy.sleep(.2)

        self.cmd_vel.publish(Twist())
        rospy.sleep(.05)

        self.cmd_vel.publish(self.move_cmd_straight)
        rospy.sleep(.05)

        self.cmd_vel.publish(Twist())
        rospy.sleep(.05)

        self.cmd_vel.publish(self.move_cmd_left)
        rospy.sleep(.05)

        self.cmd_vel.publish(Twist())
        rospy.sleep(.05)

        return "move_forward", System_state, "from right to forward"


    def correctleft_handler(self,System_state):

        self.cmd_vel.publish(self.move_cmd_left)
        rospy.sleep(0.2)

        self.cmd_vel.publish(Twist())
        rospy.sleep(.05)

        self.cmd_vel.publish(self.move_cmd_straight)
        rospy.sleep(.05)

        self.cmd_vel.publish(Twist())
        rospy.sleep(.05)

        self.cmd_vel.publish(self.move_cmd_right)
        rospy.sleep(0.05)

        self.cmd_vel.publish(Twist())
        rospy.sleep(.05)
        return "move_forward", System_state, "from left to forward"

    def terminate(self):
        # stop turtlebot
        rospy.loginfo("Stop TurtleBot")
        # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        self.cmd_vel.publish(Twist())
        # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(.1)
        rospy.loginfo("\r")
        rospy.loginfo("\r Done")
        return "End_State", ""

    def start_the_plan(self):

        m = StateMachine()
        #hell0
        rospy.loginfo('starting_the_plan')

        m.add_state("move_forward", self.move_forward_handler)
        m.add_state("correctright", self.correctright_handler)
        m.add_state("correctleft", self.correctleft_handler)
        m.add_state('clockwise',self.correct_clockwise)
        m.add_state('un_clockwise', self.correct_un_clockwise)

        m.add_state("End_state", self.terminate, end_state=1)
        m.set_start("move_forward")

        system_state = [4]  # first argument for number of cycles(*2), second for delay variable

        m.run(system_state)

    def adapt_distance(self):
        '''return the next state to go to'''

        #if CTRL+C is pressed - end state
        if rospy.is_shutdown():
            return "end_state", "finishhh"

        data = self.scanner.get_scan_data()
        avg_actual_dist=0
        for range_angle in data[85:95]:
            avg_actual_dist = avg_actual_dist+range_angle
        avg_actual_dist = avg_actual_dist/len(data[85:95])
        rospy.loginfo(avg_actual_dist)
        if avg_actual_dist < self.keep_from_wall_min:
           #rospy.loginfo('this is right')
            return "correctright" , "correcting_right"
        elif avg_actual_dist > self.keep_from_wall_max:
            #rospy.loginfo('this is left')
            return "correctleft", "correcting_left"
        else:
            #rospy.loginfo('this is forward')
            return "move_forward", "moving_forward"

    def adapt_angle(self):
        if rospy.is_shutdown():
            return "end_state", "finishhh"

        data = self.scanner.get_scan_data()
        left_data = data[70:110]
        min_value = np.min(left_data)
        min_index = left_data.index(min_value)
        rospy.loginfo(min_index)

        if min_index<15:
            return 'clockwise','adapting_clockwise'
        elif min_index>25:
            return 'un_clockwise','adapting_un_clockwise'
        else:
            return "move_forward", "moving_forward"

    def is_parallel(self):

        data = self.scanner.get_scan_data()
        left_data = data[70:110]
        min_value = np.min(left_data)
        min_index = left_data.index(min_value)
        rospy.loginfo(min_index)

        if (min_index<=25 and min_index>=15):
            return True
        else:
            return  False

if __name__ == '__main__':
    rospy.init_node('turtle_in_field', anonymous=False)
    rospy.loginfo('Main')
    robot = RunRobot()
    rospy.loginfo('robotrunned')


    robot.start_the_plan()
    robot.move_forward_handler()