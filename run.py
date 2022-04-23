from State_Machine import StateMachine
from scan import scanner
from random import random
import time
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

global scan_data

def callback(msg):
    print(msg.ranges)
    #scan_data = msg.ranges

class RunRobot:
    def __init__(self):
        # initialize

        # tell user how to stop TurtleBot
        rospy.loginfo("To stop TurtleBot CTRL + C")
        rospy.on_shutdown(self.terminate)

        # Create a publisher which can "talk" to TurtleBot and tell it to move
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)


        # Create a Subscriber which can "listen" to TurtleBot scan

        self.scan_data = None
        self.sub = rospy.Subscriber('/scan', LaserScan, callback)


        self.move_cmd_straight = Twist()
        self.move_cmd_straight.linear.x = 0.2
        self.move_cmd_straight.angular.z = 0
        self.r = rospy.Rate(10)



    def move_forward_handler(self, System_state):
        newState = "move_forward"
        transition = None
        if rospy.is_shutdown():
            return "end_state", System_state, "finishhh"

        rospy.loginfo("moving")


        # publish the velocity
        #self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.cmd_vel.publish(self.move_cmd_straight)
        # wait for 0.1 seconds (10 HZ) and publish again
        self.r.sleep()

        return newState, System_state, transition

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
        # m.add_state("Cool_on", cool_on_handler)
        # m.add_state("cool_off_delay", cool_off_delay_handler)
        # m.add_state("cool_on_delay", cool_on_delay_handler)

        m.add_state("End_state", self.terminate, end_state=1)
        m.set_start("move_forward")

        rospy.loginfo('starting_the_plan_2')

        system_state = [4]  # first argument for number of cycles(*2), second for delay variable
        m.run(system_state)




if __name__ == '__main__':
    rospy.init_node('turtle_in_field', anonymous=False)
    rospy.loginfo('Main')
    robot = RunRobot()
    rospy.loginfo('robotrunned')
    while True:
        time.sleep(1)
        #print(scan_data)
    robot.start_the_plan()