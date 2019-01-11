#Line Detection

#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from std_msgs.msg import Int16MultiArray
from cv_bridge import CvBridge, CvBridgeError
import cv2
import random
import numpy as np
import matplotlib.pyplot as plt
import math
import time

carPositionPublisher = None
timestamp = time.time()




#Distance from point (x_0,y_0) to a line by = -ax - c is given by:
#https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Line_defined_by_an_equation
#NOTE: The values b and m in this implementation refer to the standard line representation y=m*x +b
class LineModel():

    def __init__(self, point1, point2):

        #We can now calculate the slope m from the points. From that we get a
        self.m = (point1[1]-point2[1])/(point1[0]-point2[0])
        self.a = -self.m

        #We can now rearrange the formula to c= -ax -by and insert one of our points to calculate c.
        self.c = -self.a*point1[0] -point1[1]
        #NOTE: This b is not the same as in the formula! we ignore the b in the formula becuase it is 1. This b here is the offset of the function
        self.b = -self.c

        #Now we prepare the denominators of the formula's fraction for later use (it only depends on constants)
        self.denominatorPoints = pow(self.a,2)+1
        self.denominatorDist = math.sqrt(self.denominatorPoints)

    def dist(self, point):
        #Here we just apply the formula as found in the link above
        return abs(self.a*point[0] + point[1] + self.c)/self.denominatorDist

    def closestPoint(self, point):
        xVal = (point[0] - self.a*point[1] - self.a*self.c)/self.denominatorPoints
        yVal = (-self.a*point[0] + (self.a**2)*point[1] - self.c)/self.denominatorPoints
        return np.array([xVal, yVal])

    def funcValue(self, xValue):
        return (xValue*self.m +self.b)

    def antiFuncValue(self, yValue):
        return ((yValue - self.b)/self.m)

    def intersection(self, otherLine):
        xVal = (self.b-otherLine.b)/(otherLine.m-self.m)
        yVal = self.funcValue(xVal)
        return np.array([xVal, yVal])

def getOffset(img):
    midLine = img[-1]
    midWhites = np.argwhere(midLine)
    together = 0
    for i in range(0,len(midWhites)):
        if i>0 and midWhites[i][0]==midWhites[i-1][0]+1:
            together += 1
            if together>10:
                return (320 + 30 - midWhites[i - together//2][0])
        else:
            together = 0


    return None

def callback(data):
    global timestamp
    global carPositionPublisher

    time0 = time.time()
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)


    height = 480
    width = 640
    trapezBreiteOben = 200
    distSideTop = (width - trapezBreiteOben)/2
    trapezBreiteUnten = 200
    distSideBottom = (width - trapezBreiteUnten)/2

    #OL , OR , UL , UR
    src = np.float32([[distSideTop,280],[width-distSideTop,280],[distSideBottom,480],[width-distSideBottom,480]])
    dst = np.float32([[0,0],[200,0],[0, 200],[200,200]])

    gray_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray_img, 160, 255, cv2.THRESH_BINARY)

    offset = getOffset(thresh)

    carPositionPublisher.publish(str(offset))

def listener():
    global carPositionPublisher
    rospy.init_node("listener", anonymous=True)

    carPositionPublisher = rospy.Publisher("Offset", String, queue_size=10)
    rospy.Subscriber("/camera/color/image_raw", Image, callback)
    rospy.spin()

if __name__=='__main__':
    listener()
