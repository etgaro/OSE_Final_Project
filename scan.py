import rospy
from sensor_msgs.msg import LaserScan



def callback(msg):
    print(f"angle is {0}    --  distance is {66666666666666666666666}.")
    for angle in range(0,len(msg.ranges)-1):
        print(f"angle is {angle}    --  distance is {msg.ranges[angle]}.")

rospy.init_node('scan_values')
sub = rospy.Subscriber('/scan', LaserScan, callback)
rospy.spin()

