#!/usr/bin/env python
from __future__ import print_function
import rospy


if __name__=="__main__":
    try:
        rospy.init_node('Ok')
        print("Hola puto")
    except KeyboardInterrupt:
        print("TRYING TO STOPPPP")