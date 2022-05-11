
import rospy
from geometry_msgs.msg import Twist


class Rotate():
    def __init__(self):
        # initiliaze
        rospy.init_node('Rotate', anonymous=False)

        # tell user how to stop TurtleBot
        rospy.loginfo("To stop TurtleBot CTRL + C")
        rospy.loginfo("IM here and i like it !")

        # What function to call when you ctrl + c
        rospy.on_shutdown(self.shutdown)

        # Create a publisher which can "talk" to TurtleBot and tell it to move
        # Tip: You may need to change cmd_vel_mux/input/navi to /cmd_vel if you're not using TurtleBot2
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # TurtleBot will stop if we don't keep telling it to move.  How often should we tell it to move? 10 HZ
        self.r = rospy.Rate(2)

        # Twist is a datatype for velocity
        self.move_cmd = Twist()
        # let's go forward at 0.2 m/s
        self.move_cmd.linear.x = 0
        # let's turn at 0 radians/s
        self.move_cmd.angular.z = 0.5

        # as long as you haven't ctrl + c keeping doing...

        self.cmd_vel.publish(self.move_cmd)
        # wait for 0.1 seconds (10 HZ) and publish again
        for x in range(1,4):
            rospy.loginfo(x)
            self.r.sleep()

    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Stop TurtleBot")
        # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        self.cmd_vel.publish(Twist())
        # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        self.r.sleep()


if __name__ == '__main__':
    try:
        Rotate()
    except:
        rospy.loginfo("Rotate node terminated.")
