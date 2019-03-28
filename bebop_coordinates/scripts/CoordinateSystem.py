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
        self.firstTheta = None
        self.firstPosition = None
        self.odomSubs = rospy.Subscriber('odom', Odometry, self.updatePosition)
        self.rate = rospy.Rate(60)
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
        self.CMD_VEL_THREAD = threading.Thread(target=self.publishTwist)
        self.CMD_VEL_THREAD.setDaemon(True)
        self.CMD_VEL_THREAD.start()
        self.first = True

    def updatePosition(self, data):
        pos = data.pose.pose.position
        theta = data.pose.pose.orientation.z
        newPosition = np.asarray([pos.x,pos.y,pos.z])
        if self.first:
            self.firstTheta = theta
            self.firstPosition = newPosition.copy()
            self.first = False
        else:
            newPosition -= self.firstPosition
            newTheta = theta - self.firstTheta
            self.theta = newTheta
            self.position = newPosition.copy()

    def publishTwist(self):
        try:
            while not rospy.is_shutdown():
                if(self.state == "AIR"):
                    # rospy.loginfo("Publishing pose: %s"%(self.pose))
                    self.twist.linear.x = self.pose[0]
                    self.twist.linear.y = self.pose[1]
                    self.twist.linear.z = self.pose[2]
                    self.twist.angular.x = 0
                    self.twist.angular.y = 0
                    self.twist.angular.z = self.pose[3]
                    self.posePub.publish(self.twist)
                self.rate.sleep()
        except (KeyboardInterrupt, SystemExit) as e:
            print(e)
        finally:
            self.twist.linear.x = 0; self.twist.linear.y = 0; self.twist.linear.z = 0
            self.twist.angular.x = 0; self.twist.angular.y = 0; self.twist.angular.z = 0
            self.posePub.publish(self.twist)

    def RotMat(self):
        t = self.theta
        return np.asarray([
            [np.cos(t), -np.sin(t), 0],
            [np.sin(t), np.cos(t) , 0],
            [0        , 0         , 1]
        ])

    def moveTo(self, destiny): #NEEDS TO IMPLEMENT ROTATION
        try:
            destiny = np.asarray(destiny)
            print("I am at %s"%self.position)
            print("Moving to %s"%destiny)
            error = np.linalg.norm(destiny-self.position)
            counter = 1
            while (error > 0.1):
                if counter%5==0:
                    print("Destiny: %s"%destiny)
                    print("Position: %s"%self.position)
                    print("Disntace: %s"%error)
                    counter = 0
                    newPose = np.asarray([0,0,0,0])
                    rate.sleep()
                else:
                    direction = destiny - self.position
                    distance = np.linalg.norm(direction)
                    t = distance / self.speed
                    velocity = direction/t
                    newPose = np.asarray([
                        velocity[0],
                        velocity[1],
                        velocity[2],
                        self.pose[3]
                    ])
                self.pose = newPose.copy()
                error = np.linalg.norm(destiny-self.position)
                counter+=1
                self.rate.sleep()
            self.pose[0] = 0
            self.pose[1] = 0
            self.pose[2] = 0
            print("Arrived to destiny") 
        except:
            self.pose[0] = 0
            self.pose[1] = 0
            self.pose[2] = 0
    def takeoff(self):
        self.takeoffPub.publish(self.empty_msg)
        sleep(5)
        self.first = True
        self.state = "AIR"
    
    def land(self):
        self.landPub.publish(self.empty_msg)
        self.state = "LAND"

    def flattrim(self):
        self.flattrimPub.publish(self.empty_msg)
        sleep(2)