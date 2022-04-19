from State_Machine import StateMachine
from random import random
import time
import rospy
from geometry_msgs.msg import Twist


class RunRobot:
    def __init__(self):
        # initialize

        # tell user how to stop TurtleBot
        rospy.loginfo("To stop TurtleBot CTRL + C")
        rospy.on_shutdown(self.terminate)

        # Create a publisher which can "talk" to TurtleBot and tell it to move
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)


    def move_forward_handler(self, System_state):
        newState = "move_forward"
        transition = None
        if rospy.is_shutdown():
            return "end_state", System_state, "finishhh"

        rospy.loginfo("moving")
        move_cmd_straight = Twist()
        move_cmd_straight.linear.x = 0.2
        move_cmd_straight.angular.z = 0
        r = rospy.Rate(10)

        # publish the velocity
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.cmd_vel.publish(move_cmd_straight)
        # wait for 0.1 seconds (10 HZ) and publish again
        r.sleep()

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


# handler for AC turned off
# def cool_off_handler(System_state):
#     temp = 10 + 20 * random()
#     print("AC is OFF: ", "temp is", "%.2f" % temp, "cycle ", int((4-System_state[0])/2)+1)
#     time.sleep(1)
#     transition = None
#     if System_state[0] < 1: # end the simulation
#         newState = "End_state"
#     else:
#         if temp >= 20:
#             System_state[0] = System_state[0] - 1
#             newState = "cool_on_delay"
#             transition = "  Cool = on (with delay)"
#         else:
#             newState = "Cool_Off"
#     return newState, System_state, transition
#
# # handler for AC turned on
# def cool_on_handler(System_state):
#     temp = 10 + 20 * random()
#     print("AC is ON: ", "temp is", "%.2f" % temp, "cycle ", int((4-System_state[0])/2)+1)
#     time.sleep(1)
#     transition = None
#     if System_state[0] < 1: # end the simulation
#         newState = "End_state"
#     else:
#         if temp < 20:
#             System_state[0] = System_state[0] - 1
#             newState = "cool_off_delay"
#             transition = "  Cool = off (with delay)"
#         else:
#             newState = "Cool_on"
#     return newState, System_state, transition
#
# # handler for AC turned on - with delay
# def cool_on_delay_handler(System_state):
#     temp = 10 + 20 * random()
#     print("AC is ON (delay): ", "temp is", "%.2f" % temp, "cycle ", int((4-System_state[0])/2)+1,("with delay of {} seconds)".format(System_state[1]*10)))
#     time.sleep(1)
#     transition = None
#     if System_state[0] < 1: # end the simulation
#         newState = "End_state"
#     else:
#         if System_state[1] < AC_delay:
#             System_state[1] = System_state[1]+1
#             newState = "cool_on_delay"
#         elif temp < 20:
#             System_state[1] = 0
#             System_state[0] = System_state[0] - 1
#             newState = "cool_off_delay"
#             transition = "  Cool = off (with delay)"
#         else:
#             newState = "Cool_on"
#             transition = "  Cool = on"
#             System_state[1] = 0
#     return newState, System_state, transition
#
#
# # handler for AC turned off - with delay
# def cool_off_delay_handler(System_state):
#     temp = 10 + 20 * random()
#     print("AC is OFF (delay): ", "temp is", "%.2f" % temp, "cycle ", int((4-System_state[0])/2)+1,("with delay of {} seconds)".format(System_state[1]*10)))
#     time.sleep(1)
#     transition = None
#     if System_state[0] < 1: # end the simulation
#         newState = "End_state"
#     else:
#         if System_state[1] < AC_off_delay:
#             System_state[1] = System_state[1]+1
#             newState = "cool_off_delay"
#         elif temp > 20:
#             System_state[1] = 0
#             System_state[0] = System_state[0] - 1
#             newState = "cool_on_delay"
#             transition = "  Cool = on (with delay)"
#         else:
#             newState = "Cool_off"
#             transition = "  Cool = off"
#             System_state[1] = 0
#     return newState, System_state, transition
#

if __name__ == '__main__':
    rospy.init_node('turtle_in_field', anonymous=False)
    rospy.loginfo('Main')
    robot = RunRobot()
    rospy.loginfo('Main_2')

    robot.start_the_plan()