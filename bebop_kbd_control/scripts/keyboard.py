#!/usr/bin/env python

from __future__ import print_function
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
import curses

import sys
from pynput import keyboard
msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   q    w    e
   a    s    d

up key : up (+z)
down arrow : down (-z)

anything else : stop

u/j : increase/decrease max speeds by 10%
i/k : increase/decrease only linear speed by 10%
o/l : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

specialBindings = {
        'z':'takeoff',
        'x':'flattrim',
        'c':'land'
}

posMoveBindings = {
        'w':0,
        'a':1,
        'up':2,
        'q':3
}

negMoveBindings = {
        's':0,
        'd':1,
        'down':2,
        'e':3
}

speedBindings={
        'u':(1.1,1.1),
        'j':(.9,.9),
        'i':(1.1,1),
        'k':(.9,1),
        'o':(1,1.1),
        'l':(1,.9),
}

def on_press(key):
    global pose
    global takeoff
    global land
    global flattrim
    global speed
    global turn
    if(type(keyboard.Key.shift)==type(key)):
        k = key.name
    else:
        k = key.char
    if k in posMoveBindings.keys():
        pose[posMoveBindings[k]] = 1
    elif k in negMoveBindings.keys():
        pose[negMoveBindings[k]] = -1
    elif k in speedBindings.keys():
        speed = speed * speedBindings[k][0]
        turn = turn * speedBindings[k][1]
        print(vels(speed,turn))
    elif k in specialBindings.keys():
        if specialBindings[k]=='land':
            land = True
        elif specialBindings[k]=='takeoff':
            takeoff = True
        elif specialBindings[k]=='flattrim':
            flattrim = True

def on_release(key):
    global pose
    if(type(keyboard.Key.shift)==type(key)):
        k = key.name
    else:
        k = key.char
    if k in posMoveBindings.keys():
        pose[posMoveBindings[k]] = 0
    elif k in negMoveBindings.keys():
        pose[negMoveBindings[k]] = 0

def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    global pose
    global takeoff
    global land
    global flattrim
    global speed
    global turn
    takeoff = False
    land = False
    flattrim = False
    pose = [0,0,0,0]
    pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1) 
    takeoffPub = rospy.Publisher('takeoff', Empty, queue_size=1)
    flattrimPub = rospy.Publisher('flattrim', Empty, queue_size=1)
    landPub = rospy.Publisher('land', Empty, queue_size=1)
    empty_msg = Empty()
    rospy.init_node('bebop_keyboard_node')
    rate = rospy.Rate(60)
    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 1.0)

    try:
        listener = keyboard.Listener(on_press = on_press, on_release= on_release)
        listener.start()
        while(not rospy.is_shutdown()):
            print(msg)
            print(vels(speed,turn))
            twist = Twist()
            # rospy.loginfo(pose)
            twist.linear.x = pose[0]*speed
            twist.linear.y = pose[1]*speed
            twist.linear.z = pose[2]*speed
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = pose[3]*turn
            pub.publish(twist)
            if takeoff:
                takeoffPub.publish(empty_msg)
                rospy.loginfo("Taking off!")
                takeoff = False
            elif land:
                landPub.publish(empty_msg)
                rospy.loginfo("Landing")
                land = False
            elif flattrim:
                flattrimPub.publish(empty_msg)
                rospy.loginfo("Flattrim!")
                flattrim = False

            rate.sleep()

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)