import rospy
from sensor_msgs.msg import LaserScan
import numpy as np


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

    def get_x_y_from_angle_dist(self,angle,dist):
        x = -np.cos(np.radians(angle))*dist
        y = np.sin(np.radians(angle))*dist
        return x,y

    def get_generated_data(self):
        front_tree_angle, front_tree_dist, back_tree_angle,back_tree_dist = self.get_tree_angles_dist()

        front_x, front_y = self.get_x_y_from_angle_dist(90-front_tree_angle,front_tree_dist)
        back_x, back_y = self.get_x_y_from_angle_dist(back_tree_angle-90,back_tree_dist)

        m = ((front_y-back_y)/(front_x-back_x))
        b = -m*front_x+front_y

        list_of_ranges = []
        for index in range(0,40):
            x=b/(-np.tan(np.radians(index+70))-m)
            list_of_ranges.append(x/(np.sin(np.radians(index+70))))

        return list_of_ranges



    def get_avg_angle(self,data,from_ang,to_ang,step):
        count = 0
        sum_angles = 0
        tree_found = False
        for index in range(from_ang, to_ang, step):

            if data[index] == 0 and tree_found is False:
                continue

            elif data[index] == 0 and tree_found is True:
                break

            elif data[index] > 0 and tree_found is False:
                tree_found = True
                sum_angles = sum_angles + index
                count = count + 1
            elif data[index] > 0:
                sum_angles = sum_angles + index
                count = count + 1
        if step == 1 and count == 0:
            return 0

        return int(round(sum_angles/count))


    def get_tree_angles_dist(self):
        data = self.get_scan_data()
        distance_between_trees = 0.7
        for angle in data:
            if data[angle]>distance_between_trees*2:
                data[angle] = 0

        angle_front = self.get_avg_angle(data, 70, 0, -1)
        angle_back = self.get_avg_angle(data, 110, 180, 1)

        dist_front = data[angle_front]
        dist_back = data[angle_back]

        if angle_back == 0:
            angle_back = 180-angle_front
            dist_back = dist_front

        string_print = "angle_front="+str(angle_front)+"    dist_front="+str(dist_front)+"  angle_back="+str(angle_back)+"    dist_back="+str(dist_back)
        rospy.loginfo(string_print)

        return angle_front, dist_front, angle_back, dist_back






