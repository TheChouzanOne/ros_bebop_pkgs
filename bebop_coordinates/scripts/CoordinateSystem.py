#!/usr/bin/env python
from __future__ import print_function
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
import numpy as np
import threading
from time import sleep
import sys

class CoordinateSystem:
    def __init__(self, speed=0.5, turnSpeed=1):
        self.position = None
        self.theta = None
        self.velocity = None
        self.odomSubs = rospy.Subscriber('odom', Odometry, self.updatePosition)

        self.frequency = 5
        self.rate = rospy.Rate(self.frequency)
        
        self.pose = np.asarray([0,0,0,0])
        self.speed = speed
        self.turnSpeed = turnSpeed
        
        self.state = "LAND"
        
        self.posePub = rospy.Publisher('cmd_vel', Twist, queue_size=1) 
        self.takeoffPub = rospy.Publisher('takeoff', Empty, queue_size=1)
        self.flattrimPub = rospy.Publisher('flattrim', Empty, queue_size=1)
        self.landPub = rospy.Publisher('land', Empty, queue_size=1)
        
        self.empty_msg = Empty()
        self.twist = Twist()

    def updatePosition(self, data):
        position = data.pose.pose.position
        orientation = data.pose.pose.orientation
        linear = data.twist.twist.linear
        angular = data.twist.twist.angular

        self.position = np.array([position.x, position.y, position.z])
        self.orientation = np.array([orientation.x,orientation.y,orientation.z])
        self.velocity = np.array([linear.x,linear.y,linear.z,angular.z])

    def publishTwist(self):
        if(self.state == "AIR"):
            self.twist.linear.x = self.pose[0]
            self.twist.linear.y = self.pose[1]
            self.twist.linear.z = self.pose[2]
            self.twist.angular.x = 0
            self.twist.angular.y = 0
            self.twist.angular.z = self.pose[3]
            self.posePub.publish(self.twist)

    def RotMat(self):
        t = self.theta
        return np.asarray([
            [np.cos(t), -np.sin(t), 0],
            [np.sin(t), np.cos(t) , 0],
            [0        , 0         , 1]
        ])

    def moveTo(self, destiny): #NEEDS TO IMPLEMENT ROTATION
        destiny = np.asarray(destiny)
        error = np.linalg.norm(destiny-self.position)
        counter = 1
        print("Destiny: %s"%destiny)
        print("Position: %s"%self.position)
        print("Distance: %s"%error)
        while (error > 0.1):
            if counter%2==0:
                print("Destiny: %s"%destiny)
                print("Position: %s"%self.position)
                print("Pose: %s"%self.pose)
                print()
                counter = 0
                newPose = np.array([0,0,0,0])
                self.rate.sleep()
            else:
                direction = destiny - self.position
                distance = np.linalg.norm(direction)
                t = distance / self.speed
                velocity = direction/t
                newPose = np.asarray([
                    velocity[0],
                    velocity[1],
                    velocity[2],
                    0
                ])
                
            self.pose = newPose.copy()
            print("Distance: %s"%error)
            self.publishTwist()
            error = np.linalg.norm(destiny-self.position)
            counter+=1
            self.rate.sleep()

        self.brake()
        print("Arrived to destiny") 

    def brake(self, sleepTime=True):
        newPose = np.asarray([0,0,0,0])
        self.publishTwist()
        if sleepTime:
            sleep(2)

    def takeoff(self):
        self.takeoffPub.publish(self.empty_msg)
        sleep(8)
        self.first = True
        self.state = "AIR"

    def land(self):
        self.landPub.publish(self.empty_msg)
        self.state = "LAND"

    def flattrim(self):
        self.flattrimPub.publish(self.empty_msg)
        sleep(2)