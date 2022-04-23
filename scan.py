import rospy
from sensor_msgs.msg import LaserScan


class scanner():
    def __init__(self):
        #rospy.init_node('scan_values')
        self.sub = rospy.Subscriber('/scan', LaserScan, self.callback)
        self._scan = LaserScan()

    def callback(self,msg):
        print('im here')
        self._scan=msg

    def get_scan_data(self):
        return self._scan.ranges



