import rospy
from sensor_msgs.msg import LaserScan



def callback(msg):
    for angle in range(0,len(msg.ranges)-1):
        print("angle is {}  --  distance is {}".format(angle, msg.ranges[angle]))

rospy.init_node('scan_values')
sub = rospy.Subscriber('/scan', LaserScan, callback)
rospy.spin()

