from State_Machine import StateMachine
from scan import scanner
from random import random
import time
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
#from statistics import mean



class RunRobot:
    def __init__(self):
        # initialize

        # tell user how to stop TurtleBot
        rospy.loginfo("To stop TurtleBot CTRL + C")
        rospy.on_shutdown(self.terminate)

        # Create a publisher which can "talk" to TurtleBot and tell it to move
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.scanner = scanner()

        data = self.scanner.get_scan_data()
        rospy.loginfo([round(angle, 1) for angle in data[85:95]])


        # count = 0

        # while True:
        #     time.sleep(1)
        #     data = self.scanner.get_scan_data()
        #     rospy.loginfo([round(angle, 1) for angle in data[85:95]])
        #     count += 1
        #     dist = data[90]
        #     if count == 10:
        #         break

        # Create a Subscriber which can "listen" to TurtleBot scan
        # self.scan_data = None
        # self.sub = rospy.Subscriber('/scan', LaserScan, callback)

        self.move_cmd_straight = Twist()
        self.move_cmd_straight.linear.x = 0.5
        self.move_cmd_straight.angular.z = 0

        self.move_cmd_right = Twist()
        self.move_cmd_right.linear.x = 0.0
        self.move_cmd_right.angular.z = 0.5

        self.move_cmd_left = Twist()
        self.move_cmd_left.linear.x = 0.0
        self.move_cmd_left.angular.z = -0.5

        self.r = rospy.Rate(1)

        self.keep_from_wall_max = 0.40
        self.keep_from_wall_min = 0.30


    def move_forward_handler(self, System_state):

        # publish command and wait for 0.1 seconds (10 HZ)

        self.cmd_vel.publish(self.move_cmd_straight)
        self.r.sleep()

        newState, transition = self.state_to_go()

        return newState, System_state, transition

    def turn_right_handler(self,System_state):

        self.cmd_vel.publish(self.move_cmd_right)
        self.r.sleep()

        self.cmd_vel.publish(self.move_cmd_straight)
        self.r.sleep()
        self.r.sleep()
        self.r.sleep()

        self.cmd_vel.publish(self.move_cmd_left)
        self.r.sleep()

        return "move_forward", System_state, "from right to forward"


    def turn_left_handler(self,System_state):

        self.cmd_vel.publish(self.move_cmd_left)
        self.r.sleep()

        self.cmd_vel.publish(self.move_cmd_straight)
        self.r.sleep()
        self.r.sleep()
        self.r.sleep()

        self.cmd_vel.publish(self.move_cmd_right)
        self.r.sleep()


        return "move_forward", System_state, "from left to forward"

    def terminate(self):
        # stop turtlebot
        rospy.loginfo("Stop TurtleBot")
        # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        self.cmd_vel.publish(Twist())
        # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(1)
        rospy.loginfo("\r")
        rospy.loginfo("\r Done")
        return "End_State", ""

    def start_the_plan(self):

        m = StateMachine()

        rospy.loginfo('starting_the_plan')

        m.add_state("move_forward", self.move_forward_handler)
        m.add_state("turn_right", self.turn_right_handler)
        m.add_state("turn_left", self.turn_left_handler)

        m.add_state("End_state", self.terminate, end_state=1)
        m.set_start("move_forward")

        system_state = [4]  # first argument for number of cycles(*2), second for delay variable

        m.run(system_state)

    def state_to_go(self):
        '''return the next state to go to - by average range from left'''

        #if CTRL+C is pressed - end state
        if rospy.is_shutdown():
            return "end_state", "finishhh"

        data = self.scanner.get_scan_data()

        #print(len([round(angle, 1) for angle in data[65:95]]))

        avg_actual_dist=0
        for range_angle in data[65:95]:
            avg_actual_dist = avg_actual_dist+range_angle
        avg_actual_dist = avg_actual_dist/len(data[65:95])

        if avg_actual_dist < self.keep_from_wall_min:
           #rospy.loginfo('this is right')
            return "turn_right" , "turning_right"
        elif avg_actual_dist > self.keep_from_wall_max:
            #rospy.loginfo('this is left')
            return "turn_left", "turning_left"
        else:
            #rospy.loginfo('this is forward')
            return "move_forward", "moving_forward"


if __name__ == '__main__':
    rospy.init_node('turtle_in_field', anonymous=False)
    rospy.loginfo('Main')
    robot = RunRobot()
    rospy.loginfo('robotrunned')


    robot.start_the_plan()
    robot.move_forward_handler()