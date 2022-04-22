import rospy
from sensor_msgs.msg import LaserScan



def callback(msg):
    # for angle in range(0,len(msg.ranges)-1):
    #     print("angle is {}  --  distance is {}".format(angle, msg.ranges[angle]))
    return msg.ranges

rospy.init_node('scan_values')
sub = rospy.Subscriber('/scan', LaserScan, callback)

ranges = callback()

for angle in range(0,len(ranges)-1):
    print("angle is {}  --  distance is {}".format(angle, ranges[angle]))
rospy.spin()

