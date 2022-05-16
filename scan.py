import rospy
from sensor_msgs.msg import LaserScan


class scanner():
    def __init__(self):
        #rospy.init_node('scan_values')
        self.sub = rospy.Subscriber('/scan', LaserScan, self.callback)
        self.scan = LaserScan()
        rospy.sleep(1)

    def callback(self,msg):
        print("hey")
        self.scan=msg

    def get_scan_data(self):
        return self.scan.ranges



