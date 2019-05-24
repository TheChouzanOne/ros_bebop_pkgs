#!/usr/bin/env python
from __future__ import print_function
import sys
import rospy
import numpy as np
from time import sleep
from geometry_msgs.msg import Twist
sys.path.insert(0, '/home/adrian/Documents/CETYS/Vehiculos/bebop_ws/src/bebop_autonomy/bebop_coordinates/scripts')
from CoordinateSystem import CoordinateSystem


def updateDestiny(data):
    global STATE
    global inverted
    linear = data.linear
    x = linear.x
    y = linear.y
    z = linear.z # POR ALGUNA RAZON LOS VALORES DE DATA NO SE ACTUALIZAN A PESAR DE QUE SI SE PUBLICAN DE FORMA CORRECTA
    print("X: %s\nY: %s\nZ: %s"%(x,y,z))
    if(inverted):
        y *= -1
        x *= -1
    if(STATE=="CENTERING"):
        if(np.linalg.norm(np.asarray([y, z])) < 0.05):
            STATE = "ADVANCING"
        else:
            STATE = "MOVING"
            destiny = np.asarray([CS.position[0], CS.position[1]+y, CS.position[2]+z])
            print("Position: %s"%CS.position)
            CS.moveTo(destiny)
            STATE = "CENTERING"
    elif(STATE=="ADVANCING"):
        if(x < 1.5):
            offx = 2.3
            offy = 1.5
            if inverted:
                offx *= -1
                offy *= -1
            # CS.moveTo([CS.position[0]+2, CS.position[1], CS.position[2]])
            destiny = np.asarray([CS.position[0]+offx, CS.position[1], CS.position[2]])
            CS.moveTo(destiny)
            # CS.moveTo([CS.position[0], CS.position[1]-2, CS.position[2]])
            CS.rotate()
            inverted = CS.inverted

        else:
            offx = 0.3
            if inverted:
                offx *= -1
            destiny = [CS.position[0]+offx, CS.position[1], CS.position[2]]
            CS.moveTo(destiny)
        STATE = "CENTERING"

if __name__=="__main__":
    try:
        rospy.init_node('bebop_flight_node')
        global STATE
        global CS
        global inverted
        inverted = False
        CS = CoordinateSystem(speed=0.5, turnSpeed=1)
        CS.flattrim()
        CS.takeoff()
        origin = CS.position.copy()
        print("Origin is %s"%origin)
        STATE = "CENTERING" #CENTERING | ADVANCING | ROTATING
        destinySubs = rospy.Subscriber('/bebop/destiny', Twist, updateDestiny)
        if not rospy.is_shutdown():
            while(True):
                print("STATE: %s"%STATE)
                CS.rate.sleep()

    except KeyboardInterrupt:
        print("TRYING TO STOPPPP")
        CS.land()