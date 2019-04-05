#!/usr/bin/env python

from __future__ import print_function
import sys
import rospy
import numpy as np
from time import sleep
sys.path.insert(0, '/home/adrian/Documents/CETYS/Vehiculos/bebop_ws/src/bebop_autonomy/bebop_coordinates/scripts')
from CoordinateSystem import CoordinateSystem

def circlePoints(n, r):
    initialP = np.array([r,0,0], dtype='float64')
    theta = np.pi * 2 / n
    points = list()

    rotMat = np.asarray([
        [np.cos(theta), -np.sin(theta), 0],
        [np.sin(theta), np.cos(theta) , 0],
        [0            , 0             , 1]
    ])
    for i in range(n):
        initialP = np.matmul(rotMat, initialP)
        points.append(initialP)
    return points

if __name__=="__main__":
    rospy.init_node('bebop_circle_node')
    try:
        if not rospy.is_shutdown():
            CS = CoordinateSystem(speed=0.5, turnSpeed=1)
            points = circlePoints(6, 0.6)
            CS.flattrim()
            CS.takeoff()
            origin = CS.position.copy()
            print("Origin is %s"%origin)    
            # CS.moveTo(origin + np.asarray([0, 1, 0]))
            # CS.moveTo(origin)
            for p in points:
                print("Origin is %s"%origin)
                CS.moveTo(p+origin)
            CS.land()

    except KeyboardInterrupt:
        print("TRYING TO STOPPPP")
        CS.land()