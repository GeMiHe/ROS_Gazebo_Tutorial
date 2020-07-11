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
from sensor_msgs.msg import Temperature
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from collections import deque
from commander.msg import FireSource

class CircularBuffer(deque): 
    def __init__(self, size=0): 
        super(CircularBuffer, self).__init__(maxlen=size) 
    
    @property 
    def average(self): 
        return sum(self)/len(self)

    def increasing(self):
        return self[-1] >= self[0]
 
 
class TempSearch(): 
 
    def __init__(self): 
        self.counter = 1
        self.secondFound = False
        self.firstFound = False
        self.escaping = False
        self.maxTemp = [[0,0,0],[0,0,0]]
        self.maxBufferLength = rospy.get_param('tempSearch/maxBufferLength')
        self.maxBuffer = CircularBuffer(self.maxBufferLength)
        for i in range(self.maxBufferLength):
            self.maxBuffer.append(random.randint(0,1000))
        self.tempBuffer = CircularBuffer(2)
        # Init node and load params 
        self.maxVel = rospy.get_param('tempSearch/max_speed')
        self.maxTurn = rospy.get_param('tempSearch/max_turnspeed')
        rospy.init_node('temp_search') 
        rospy.Subscriber("/odom", Odometry, self.odomCallback)
        rospy.Subscriber("/temperature", Temperature, self.tempCallback)
        self.twistPublisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.fireSourcePublisher = rospy.Publisher("/fire_source", FireSource, queue_size=1)
        rospy.spin()

    def odomCallback(self, data):
        self.odom = data
        #rospy.loginfo(data)
        #twistMsg = Twist()
        #twistMsg.linear.x = 0
        #self.twistPublisher.publish(twistMsg)

    def tempCallback(self, data):
        if self.secondFound:
            print(self.maxTemp)
            twistMsg = Twist()
            self.twistPublisher.publish(twistMsg)
            return
        twistMsg = Twist()
        self.tempBuffer.append(data.temperature)
        if not self.firstFound:
            print("searching first")
            if self.tempBuffer.increasing():
                twistMsg.linear.x = self.maxVel
            else:
                twistMsg.angular.z = self.maxTurn
            if data.temperature > 100 and self.maxTemp[0][0] < data.temperature:
                self.maxTemp[0] = [data.temperature, self.odom.pose.pose.position.x, self.odom.pose.pose.position.y]
            elif data.temperature > 100:
                self.maxBuffer.append(self.maxTemp[0][0])
                if round(self.maxBuffer.average,2) == round(self.maxTemp[0][0],2):
                    self.firstFound = True
                    self.escaping = True
                    fireSourceMsg = FireSource()
                    fireSourceMsg.header.frame_id = "first_fire_source"
                    fireSourceMsg.temperature = Temperature()
                    fireSourceMsg.temperature.temperature = self.maxTemp[0][0]
                    fireSourceMsg.pose = Pose()
                    fireSourceMsg.pose.position.x = self.maxTemp[0][1]
                    fireSourceMsg.pose.position.y = self.maxTemp[0][2]
                    self.fireSourcePublisher.publish(fireSourceMsg)
        elif self.escaping:
            print("escaping")
            if math.sqrt(((self.odom.pose.pose.position.x-self.maxTemp[0][1])**2)+((self.odom.pose.pose.position.y-self.maxTemp[0][2])**2)) < self.counter:
                twistMsg.linear.x = self.maxVel
            else:
                for i in range(self.maxBufferLength):
                    self.maxBuffer.append(random.randint(0,1000))
                self.counter += 1
                self.escaping = False
        else:
            print("searching second")
            if self.tempBuffer.increasing():
                twistMsg.linear.x = self.maxVel
            else:
                twistMsg.angular.z = self.maxTurn
            if data.temperature > 100 and self.maxTemp[1][0] < data.temperature:
                self.maxTemp[1] = [data.temperature, self.odom.pose.pose.position.x, self.odom.pose.pose.position.y]
            elif data.temperature > 100:
                self.maxBuffer.append(self.maxTemp[1][0])
                if round(self.maxBuffer.average,2) == round(self.maxTemp[1][0],2) and math.sqrt(((self.maxTemp[1][1]-self.maxTemp[0][1])**2)+((self.maxTemp[1][2]-self.maxTemp[0][2])**2)) > 2:
                    self.secondFound = True
                    fireSourceMsg = FireSource()
                    fireSourceMsg.header.frame_id = "second_fire_source"
                    fireSourceMsg.temperature = Temperature()
                    fireSourceMsg.temperature.temperature = self.maxTemp[1][0]
                    fireSourceMsg.pose = Pose()
                    fireSourceMsg.pose.position.x = self.maxTemp[1][1]
                    fireSourceMsg.pose.position.y = self.maxTemp[1][2]
                    self.fireSourcePublisher.publish(fireSourceMsg)
                elif round(self.maxBuffer.average,2) == round(self.maxTemp[1][0],2) and math.sqrt(((self.maxTemp[1][1]-self.maxTemp[0][1])**2)+((self.maxTemp[1][2]-self.maxTemp[0][2])**2)) <= self.counter:
                    self.escaping = True
                    if self.maxTemp[1][0] > self.maxTemp[0][0]:
                        self.maxTemp[0] = self.maxTemp[1]
                        fireSourceMsg = FireSource()
                        fireSourceMsg.header.frame_id = "first_fire_source"
                        fireSourceMsg.temperature = Temperature()
                        fireSourceMsg.temperature.temperature = self.maxTemp[0][0]
                        fireSourceMsg.pose = Pose()
                        fireSourceMsg.pose.position.x = self.maxTemp[0][1]
                        fireSourceMsg.pose.position.y = self.maxTemp[0][2]
                        self.fireSourcePublisher.publish(fireSourceMsg)
                    self.maxTemp[1] = [0,0,0]
        self.twistPublisher.publish(twistMsg)
        #print(self.maxTemp)
        #rospy.loginfo(data.temperature)

    
if __name__ == '__main__': 

    try:
        TempSearch() 

    except rospy.ROSInterruptException: 
        rospy.loginfo("Search finished because of some failures.") 
