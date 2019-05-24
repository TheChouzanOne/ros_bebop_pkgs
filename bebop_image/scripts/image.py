#!/usr/bin/env python
from __future__ import print_function
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from pynput import keyboard
import cv2
import rospy
import sys
import numpy as np
import imutils


# image shape 480x856 hxw


class videoFeed:
    def __init__(self):
        self.bridge = CvBridge()
        print("Listening...")
        self.hReal = 1.0
        self.knownDistance = (float(self.hReal/2.0)) * \
            np.sin(self.toRadian(65))/np.sin(self.toRadian(25))
        self.squarePxHeight = 480
        self.WIDTH_CENTER = 856/2
        self.HEIGHT_CENTER = 480/2
        self.alpha = 1
        self.beta = 50
        self.visualData = Twist()
        self.publisher = rospy.Publisher('/bebop/destiny', Twist, queue_size=1)
        rospy.Subscriber("/bebop/image_raw", Image, self.getTarget)
        listener = keyboard.Listener(on_press=self.onPress)
        listener.start()
        rospy.spin()

    def toRadian(self, angle):
        return angle * np.pi/180

    def getDistance(self, width):
        return (self.squarePxHeight * self.knownDistance) / width

    def getTarget(self, data):
        image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, (0, 50, 50), (10, 255, 255))
        mask2 = cv2.inRange(hsv, (170, 50, 50), (180, 255, 255))
        mask = cv2.bitwise_or(mask1, mask2)

        bright = cv2.addWeighted(image, self.alpha, np.zeros(
            image.shape, image.dtype), 0, self.beta)
        cv2.putText(image, 'Alpha: {0}'.format(
            self.alpha), (750, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        blur = cv2.GaussianBlur(bright, (5, 5), 0)
        output = cv2.bitwise_and(blur, blur, mask=mask)

        gray = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
        gaussian = cv2.GaussianBlur(gray, (5, 5), 0)
        ret3, otsu = cv2.threshold(
            gaussian, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)

        edge = cv2.Canny(otsu, 30, 200)
        img, contours, hierarchy = cv2.findContours(
            edge, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        square = []
        for contour in contours:
            if cv2.contourArea(contour) > 5000:
                approx = cv2.convexHull(contour)
                approx = cv2.approxPolyDP(
                    approx, 0.07*cv2.arcLength(contour, True), True)
                if ((len(approx) == 4)):
                    length = cv2.arcLength(contour, True)/4
                    M = cv2.moments(contour)
                    try:
                        cx = int(M['m10']/M['m00'])
                        cy = int(M['m01']/M['m00'])
                        distance = self.getDistance(length)
                        cv2.circle(image, (cx, cy), (cx/100), (0, 255, 0), -1)
                        #calcular desfase
                        y = (self.WIDTH_CENTER - cx)/length
                        z = (self.HEIGHT_CENTER - cy)/length
                        self.visualData.linear.x = distance
                        self.visualData.linear.y = y
                        self.visualData.linear.z = z

                        cv2.putText(image, 'Distance to target ({:.2f}, {:.2f}, {:.2f}) m'.format(
                            distance, y ,z), (cx - 100, int(cy-length/2)-20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                        self.publisher.publish(self.visualData)
                    except ZeroDivisionError:
                        #PONER AQUI UN PUBLISH PARA CUANDO YA NO SE DETECTA
                        print("Contour is probably too tiny")
                    square.append(approx)
        cv2.drawContours(image, square, -1, (0, 255, 0), 3)
        cv2.imshow('img', image)
        cv2.waitKey(1)

    def onPress(self, key):
        if(str(key) == 'Key.up'):
            self.alpha += 0.1
        elif(str(key) == 'Key.down'):
            self.alpha -= 0.1


if __name__ == "__main__":
    try:
        rospy.init_node('bebop_image_node')
        vf = videoFeed()
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
        print("Shutting down...")