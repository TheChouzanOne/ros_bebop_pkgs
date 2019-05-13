#!/usr/bin/env python

from __future__ import print_function
import sys
import rospy
import numpy as np
from time import sleep
sys.path.insert(0, '/home/adrian/Documents/CETYS/Vehiculos/bebop_ws/src/bebop_autonomy/bebop_coordinates/scripts')
from CoordinateSystem import CoordinateSystem


if __name__=="__main__":
    rospy.init_node('bebop_circle_node')
    try:
        if not rospy.is_shutdown():
            CS = CoordinateSystem(speed=0.5, turnSpeed=1)
            CS.flattrim()
            CS.takeoff()
            CS.moveTo([1, 0, CS.initialHeight])
            CS.moveTo([0, 0, CS.initialHeight])
            CS.land()

    except KeyboardInterrupt:
        print("TRYING TO STOPPPP")
        CS.land()