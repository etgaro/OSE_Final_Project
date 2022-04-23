import rospy
from sensor_msgs.msg import LaserScan


class scanner():
    def __init__(self):
        #rospy.init_node('scan_values')
        rospy.Subscriber('/scan', LaserScan, self.callback)
        rospy.spin()

    def callback(self, msg):
        # for angle in range(0,len(msg.ranges)-1):
        #     print("angle is {}  --  distance is {}".format(angle, msg.ranges[angle]))
        return msg.ranges



