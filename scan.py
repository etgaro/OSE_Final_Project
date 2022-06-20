import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
from Exceptions import NoFrontTreeError

#this is the scanner class, all functions and actions related to the Lidar sensor appear here.

class scanner():
    def __init__(self):
        #rospy.init_node('scan_values')
        self.sub = rospy.Subscriber('/scan', LaserScan, self.callback)
        self.scan = LaserScan()
        rospy.sleep(1) #need to give the scanner a second to start sending back info

    def callback(self,msg):
        #print("hey")
        self.scan=msg

    #functions that returns an array of ranges, every index is a degree - so 360 ranges in total
    def get_scan_data(self):
        return self.scan.ranges

    #function used in the generated data function - for a given angle and distanse this function returns the
    #(x,y) coordinates of an object - in relation to the robot
    def get_x_y_from_angle_dist(self,angle,dist):
        x = -np.cos(np.radians(angle))*dist
        y = np.sin(np.radians(angle))*dist
        return x,y

    #this function returns an array containg the ranges of the robot from a line between 2 trees. the array
    #includes the ranges at the angles of 70-110 degrees
    def get_generated_data(self):
        #first, need to find a tree in from and a tree in back
        front_tree_angle, front_tree_dist, back_tree_angle,back_tree_dist = self.get_tree_angles_dist()

        #next, need to set the (X,Y) coordinates of each tree
        front_x, front_y = self.get_x_y_from_angle_dist(90-front_tree_angle,front_tree_dist)
        back_x, back_y = self.get_x_y_from_angle_dist(back_tree_angle-90,back_tree_dist)

        #adjusting the back Y to match the real location
        back_y = -back_y

        #checking if the slope on the line between the 2 trees is zero so we can avoid deviding by zero.
        if front_x-back_x == 0:
            m=0
        else:
            m = ((front_y-back_y)/(front_x-back_x))

        # setting the equation for the line between the trees
        b = -m*front_x+front_y

        list_of_ranges = []
        for index in range(0,40):
            if index+70 == 90: # checking the 90 degrees
                if m==0: # parallel to trees line
                    x= front_x
                    string_to_print = "paralel --- x= front_x = " + str(x) + "b="+str(b)+"front_y="+str(front_y)
                    rospy.loginfo(string_to_print)
                else:
                    x=b/m
                    string_to_print = "x=b/m = "+str(x)
                    rospy.loginfo(string_to_print)
                list_of_ranges.append(abs(x))

            else:
                if m==0: #the robot is parallel to trees line, calculate range using the front_x
                    if index+70<90:
                     list_of_ranges.append(abs(front_x / (np.sin(np.radians(index + 70)))))
                    else:
                     list_of_ranges.append(abs(front_x / (np.sin(np.radians(180 - (index + 70))))))
                else: #calculate range using x calculated through meeting point of the line of trees and the line
                      #created between the robot and tree line at the current angel
                    x=b/(np.tan(np.radians(90+index+70))-m)
                    if index+70<90:
                     list_of_ranges.append(abs(x / (np.sin(np.radians(index + 70)))))
                    else:
                     list_of_ranges.append(abs(x / (np.sin(np.radians(180 - (index + 70))))))

        return list_of_ranges


    #function that searches for an object either in the front left or back left of the robot - depending on the step
    def get_avg_angle(self,data,from_ang,to_ang,step):
        count = 0
        sum_angles = 0
        tree_found = False
        #loop looking for a tree in the data array, if the range listed is larger than 0 that means there is something there
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
        #if we are searching in back (step = 1) and no tree has been found - return 0
        if step == 1 and count == 0:
            return 0
        try:
            return int(round(sum_angles/count))
        except ZeroDivisionError: #if there is devision by zero that means no tree has been found
            raise NoFrontTreeError

    #function the finds the angles and distance of trees in the row
    def get_tree_angles_dist(self):
        data = self.get_scan_data()
        distance_between_trees = 0.6

        data = list(data)
        #clearing up the ranges data so as to eliminate "noise" if something is recognized too far away = we turn it to 0
        for angle in range(0,180,1):
            if data[angle]>distance_between_trees*1.5:
                data[angle] = 0

        angle_front = self.get_avg_angle(data, 70, 0, -1) #the front tree will be between 0 and 70 degrees
        angle_back = self.get_avg_angle(data, 110, 180, 1) #the back tree will be between 110 and 180 degrees

        dist_front = data[angle_front]
        dist_back = data[angle_back]

        if angle_back == 0: #havent found tree in back - meaning we are at the beginning of a row
            #mirror the front tree to back
            angle_back = 180-angle_front
            dist_back = dist_front

        string_print = "angle_front="+str(angle_front)+"    dist_front="+str(dist_front)+"  angle_back="+str(angle_back)+"    dist_back="+str(dist_back)
        rospy.loginfo(string_print)

        return angle_front, dist_front, angle_back, dist_back

    #search for a tree from the left using real scanner data
    def tree_from_side(self):
        left_real_data = self.get_scan_data()
        distance_from_side = min(left_real_data[88:92])
        #checking if the object found is within 1 meter of the robot
        if distance_from_side < 1 and distance_from_side>0:
            return True
        return False




