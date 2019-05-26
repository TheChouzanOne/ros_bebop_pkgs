#!/usr/bin/env python
from __future__ import print_function
import sys
import rospy
import numpy as np
from time import sleep
from geometry_msgs.msg import Twist
sys.path.insert(0, '/home/adrian/Documents/CETYS/Vehiculos/bebop_ws/src/bebop_autonomy/bebop_coordinates/scripts')
from CoordinateSystem import CoordinateSystem
from simple_pid import PID

def updateDestiny(data):
    global pidX, pidY, pidZ, STATE, CS, count
    linear = data.linear
    x = linear.x
    y = linear.y
    z = linear.z # POR ALGUNA RAZON LOS VALORES DE DATA NO SE ACTUALIZAN A PESAR DE QUE SI SE PUBLICAN DE FORMA CORRECTA
    velX, velY, velZ = pidX(x), pidY(y), pidZ(z)
    print("STATE: %s"%STATE)
    print("X: %s\tY: %s\tZ: %s"%(x,y,z))
    print("velX: %s\tvelY: %s\tvelZ: %s"%(velX,velY,velZ))
    print("Position: %s"%CS.position)
    print()
    planeError = np.linalg.norm(np.asarray([velY, velZ]))
    if(STATE=="CENTERING"):
        if(x == -100):
            count += 1
            if(count > 200):
                STATE = "ADVANCING"
                CS.state = "AIR"
                if(CS.inverted):
                    CS.moveTo(CS.position + np.asarray([3,0,0]))
                    # CS.moveTo(CS.position + np.asarray([0, -1.5, 0]))
                    # CS.moveTo(CS.position + np.asarray([-0.5,0,0]))
                    pass
                else:
                    CS.moveTo(CS.position + np.asarray([-3,0,0]))
                    # CS.moveTo(CS.position + np.asarray([0, 1.5, 0]))
                    # CS.moveTo(CS.position + np.asarray([0.5,0,0]))    
                    pass
                CS.rotate()
                CS.state = "local"
                STATE = "CENTERING"
        else:
            count = 0
            if(planeError > 0.05):
                publishTwist(0, velY, velZ)
                pass
            elif(x > 1.15):
                publishTwist(velX, 0, 0)
                pass
    
def publishTwist(x, y, z):
    global posePub
    twist = Twist()
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0
    twist.linear.x = x
    twist.linear.y = y
    twist.linear.z = z
    posePub.publish(twist)

try:
    rospy.init_node('bebop_flight_node')
    count = 0
    inverted = False
    CS = CoordinateSystem(speed=0.5, turnSpeed=1)
    CS.flattrim()
    CS.takeoff()
    origin = CS.position.copy()
    STATE = "CENTERING"
    CS.state = "local"
    pidX = PID(-0.4, 0, -1, setpoint=0, output_limits=(-CS.speed,CS.speed))
    pidY = PID(-0.4, 0, -1, setpoint=0, output_limits=(-CS.speed,CS.speed))
    pidZ = PID(-0.4, 0, -1, setpoint=0, output_limits=(-CS.speed,CS.speed))
    print("Origin is %s"%origin)
    STATE = "CENTERING" #CENTERING | ADVANCING | ROTATING
    destinySubs = rospy.Subscriber('/bebop/destiny', Twist, updateDestiny)
    posePub = rospy.Publisher('cmd_vel', Twist, queue_size=1) 
    while(not rospy.is_shutdown()):
        CS.rate.sleep()

except KeyboardInterrupt:
    print("TRYING TO STOPPPP")
    CS.land()
