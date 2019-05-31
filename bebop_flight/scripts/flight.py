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

def isInverted():
    global CS
    return CS.inverted != 1

def updateDestiny(data):
    global pidX, pidY, pidZ, STATE, CS, count, marcos
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
            if(count > 100):
                if(STATE=="ADVANCING"):
                    return #To kill any other thread entering
                STATE = "ADVANCING"
                CS.state = "AIR"
                if(isInverted()):
                    print("Entro al primero")
                    CS.moveTo(CS.position + np.asarray([-2.5,0,0]))
                    CS.moveTo(CS.position + np.asarray([0, 3, 0])) #CAMBIAR ESTO A 3.5 !!!!!!!!!!!!!!!!!! 
                    CS.moveTo(CS.position + np.asarray([2,0,0]))    
                    pass
                else:
                    print("Entro al segundo")
                    CS.moveTo(CS.position + np.asarray([2.5,0,0]))
                    CS.moveTo(CS.position + np.asarray([0, -3, 0]))
                    CS.moveTo(CS.position + np.asarray([-1,0,0]))
                    pass
                if(marcos == 0):
                    CS.rotate()
                marcos += 1
                CS.state = "local"
                sleep(2)
                STATE = "CENTERING"
        else:
            count = 0
            publishTwist(velX, velY, velZ)
            # if(planeError > 0.05):
            #     publishTwist(0, velY, velZ)
            #     pass
            # elif(x > 1.5):
            #     publishTwist(velX, 0, 0)
            #     pass
    
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
    CS = CoordinateSystem(speed=0.2, turnSpeed=1)
    CS.flattrim()
    CS.takeoff()
    origin = CS.position.copy()
    STATE = "CENTERING"
    CS.state = "local"
    marcos = 0
    pidX = PID(-0.03, 0, 0, setpoint=0, output_limits=(-CS.speed/2,CS.speed/2))
    pidY = PID(-0.2, 0, 0, setpoint=0, output_limits=(-CS.speed,CS.speed))
    pidZ = PID(-0.2, 0, 0, setpoint=0, output_limits=(-CS.speed,CS.speed))
    print("Origin is %s"%origin)
    STATE = "CENTERING" #CENTERING | ADVANCING | ROTATING
    destinySubs = rospy.Subscriber('/bebop/destiny', Twist, updateDestiny)
    posePub = rospy.Publisher('cmd_vel', Twist, queue_size=1) 
    while(not rospy.is_shutdown()):
        if(marcos > 1):
            STATE = "LANDING"
            print("Marcos cruzados: %s"%marcos)
            # CS.moveTo(origin)
            # CS.rotError = 0.005
            # CS.rotate()
            CS.land()
            break
        CS.rate.sleep()

except KeyboardInterrupt:
    print("TRYING TO STOPPPP")
    CS.land()
