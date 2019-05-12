#!/usr/bin/env python
from __future__ import print_function
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from simple_pid import PID
import numpy as np
import threading
from time import sleep
import sys

class CoordinateSystem:
    def __init__(self, speed=0.5, turnSpeed=1):
        self.origin = None
        self.position = None
        self.destiny = None
        self.velocity = None
        self.odomSubs = rospy.Subscriber('odom', Odometry, self.updatePosition)


        self.pid = list()
        self.Kp = 0.4
        self.Ki = 0
        self.Kd = 1

        self.frequency = 10
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

        self.MoveThread = threading.Thread(target=self.PIDMove)
        self.MoveThread.setDaemon(True)


    def updatePosition(self, data):
        position = data.pose.pose.position
        orientation = data.pose.pose.orientation
        linear = data.twist.twist.linear
        angular = data.twist.twist.angular

        self.position = np.array([position.x, position.y, position.z])
        self.orientation = np.array([orientation.x,orientation.y,orientation.z])
        self.velocity = np.array([linear.x,linear.y,linear.z,angular.z])

    def publishTwist(self):
        self.twist.linear.x = self.pose[0]
        self.twist.linear.y = self.pose[1]
        self.twist.linear.z = self.pose[2]
        self.twist.angular.x = 0
        self.twist.angular.y = 0
        self.twist.angular.z = self.pose[3]
        self.posePub.publish(self.twist)

    def RotMat(self, t):
        return np.asarray([
            [np.cos(t), -np.sin(t), 0],
            [np.sin(t), np.cos(t) , 0],
            [0        , 0         , 1]
        ])

    def PIDMove(self):
        error = np.zeros(3)
        while(True):
            if("AIR"):
                newPose = np.zeros(4)
                for i in range(3):
                    error[i] = newPose[i] = self.pid[i](self.position[i])            
                self.pose = newPose.copy()
                print("Velocity: %s"%np.linalg.norm(error))
                self.publishTwist()
                self.rate.sleep()
            else:
                pass

    def setPIDDestiny(self, destiny):
        for i in range(3):
            self.pid[i].setpoint = destiny[i]

    def moveTo(self, destiny): #NEEDS TO IMPLEMENT ROTATION
        destiny = np.asarray(destiny)
        self.rate.sleep()
        self.setPIDDestiny(destiny)
        err = np.linalg.norm(destiny-self.position)
        while (err > 0.1):
            self.rate.sleep()
            err = np.linalg.norm(destiny-self.position)
            print("\nDistance: %s"%err)
        print("Arrived to destiny") 

    def brake(self, sleepTime=True):
        newPose = np.asarray([0,0,0,0])
        self.publishTwist()
        if sleepTime:
            sleep(2)

    def takeoff(self):
        self.takeoffPub.publish(self.empty_msg)
        self.origin = self.position.copy()
        sleep(8)
        current = self.position.copy()
        for i in range(3):
            self.pid.append(PID(self.Kp, self.Ki, self.Kd, setpoint=current[i], output_limits=(-self.speed,self.speed)))
        self.state = "AIR"
        self.MoveThread.start()

    def land(self):
        self.state = "LAND"
        self.rate.sleep()
        self.landPub.publish(self.empty_msg)

    def flattrim(self):
        self.flattrimPub.publish(self.empty_msg)
        sleep(2)