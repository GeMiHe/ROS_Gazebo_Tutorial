#!/usr/bin/env python 
 
import rospy 
import math 
import random

 
import actionlib 
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal 
from actionlib_msgs.msg import GoalStatusArray 
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from tf.transformations import quaternion_from_euler 
from std_srvs.srv import Empty 
from nav_msgs.msg import Odometry

from visualization_msgs.msg import Marker

from pal_person_detector_opencv.msg import Detections2d
 
 
class PersonSearch(): 
 
    def __init__(self): 
        self.points = [[2.35,-1.9, False],[0.25,-2.9, False],[-1.85,-1.6, False],[-1.8,0.85, False],[-1.65,3.75, False],[0.3,2.55, False],[0.5,4.25, False],[1.85,2.6, False],[2.5,0.7, False]]
        self.person = None
        self.ind = 0
        pubRate = 1
        # Init node and load params 
        rospy.Subscriber("/person_detector/detections", Detections2d, self.detectCallback)
        rospy.Subscriber("/odom", Odometry, self.odomCallback)
        rospy.Subscriber("/move_base/status", GoalStatusArray, self.goalStatusCallback)
        self.goalPublisher = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        self.markerPublsher = rospy.Publisher('/missing_person', Marker, queue_size=1)
        rospy.init_node('person_search') 
        self.rate = rospy.Rate(pubRate) 
 
        while self.person == None: 
            if not self.points[self.ind][2]:
                goal = PoseStamped()
                goal.header.frame_id = "map" 
                goal.header.stamp = rospy.Time.now() 
                goal.pose.position.x = self.points[self.ind][0]
                goal.pose.position.y = self.points[self.ind][1]
                goal.pose.orientation = Quaternion(*quaternion_from_euler(0.0, 0.0, math.radians(90.0)))
                self.goalPublisher.publish(goal)
            else:
                self.ind += 1

            self.rate.sleep()

    def goalStatusCallback(self, data):
        if data.status_list and data.status_list[0].text == "Goal reached." and math.sqrt((self.odom.pose.pose.position.x-self.points[self.ind][0])**2 + (self.odom.pose.pose.position.y-self.points[self.ind][1])**2) < 1:
            self.points[self.ind][2] = True

    def detectCallback(self, data):
        #print(data)
        if len(data.detections) > 0:
            self.person = [data.detections[0].x, data.detections[0].y]
            personMarker = Marker()
            personMarker.header.frame_id = "missing_person"
            personMarker.header.stamp = rospy.Time.now()
            personMarker.pose = self.odom.pose.pose
            self.markerPublsher.publish(personMarker)
        print(self.person)

    def odomCallback(self, data):
        self.odom = data

    
if __name__ == '__main__': 

    try:
        PersonSearch() 

    except rospy.ROSInterruptException: 
        rospy.loginfo("Search finished because of some failures.") 
