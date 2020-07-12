#!/usr/bin/env python 
 
import rospy 
import math 
import random
 
import actionlib 
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal 
from actionlib_msgs.msg import GoalStatus 
from geometry_msgs.msg import Pose, Point, Quaternion 
from tf.transformations import quaternion_from_euler 
from std_srvs.srv import Empty 
from nav_msgs.msg import Odometry

from pal_person_detector_opencv.msg import Detections2d
 
 
class PersonSearch(): 
 
    def __init__(self): 
        self.person = None
        pubRate = 1
        # Init node and load params 
        rospy.Subscriber("/person_detector/detections", Detections2d, self.detectCallback)
        rospy.Subscriber("/odom", Odometry, self.odomCallback)
        rospy.init_node('person_search') 
        self.rate = rospy.Rate(pubRate) 
 
        while not rospy.is_shutdown(): 
            goal = MoveBaseGoal() 
            goal.target_pose.header.frame_id = "map" 
            goal.target_pose.header.stamp = rospy.Time.now() 
            goal.target_pose.pose = self.poses[index] 
            
            self.client.send_goal(goal, self.doneCB, self.activeCB, self.feedbackCB)

        self.rate.sleep()

    def detectCallback(self, data):
        #print(data)
        if len(data.detections) > 0:
            self.person = [data.detections[0].x, data.detections[0].y]
        print(self.person)

    def odomCallback(self, data):
        self.odom = data

    
if __name__ == '__main__': 

    try:
        PersonSearch() 

    except rospy.ROSInterruptException: 
        rospy.loginfo("Search finished because of some failures.") 
