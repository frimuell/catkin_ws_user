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


def ransac(whiteDotSet, percentageThreshold=0.2, distanceThreshold=20.0, iterations=30):
    bestCandidateSet = None
    antiSetOfBestSet = None
    bestModel = None
    #Percentage means percentage of dots inside the distance threshold of a model line
    bestPercentage = 0

    for j in range(0,iterations):
        twoRands = np.random.randint(0, len(whiteDotSet),2)
        #Haven't found a better way to avoid choosing the same point twice
        if twoRands[0] == twoRands[1]:
            continue

        dotA, dotB = whiteDotSet[twoRands[0]], whiteDotSet[twoRands[1]]

        #A model for a line that goes through dotA and dotB
        ln = LineModel(dotA, dotB)

        def closeEnough(dot):
            return ln.dist(dot)<distanceThreshold

        #Using numpy feature "Boolean Indexing"
        boolSet = np.apply_along_axis(func1d=closeEnough, axis=1, arr=whiteDotSet)
        consensusSet= whiteDotSet[boolSet]


        percentageActual=len(consensusSet)/float(len(whiteDotSet))

        #if(percentageActual>percentageThreshold and percentageActual>bestPercentage):
        if(percentageActual>bestPercentage):
            bestCandidateSet = consensusSet
            antiSetOfBestSet = whiteDotSet[np.logical_not(boolSet)]
            bestModel = ln
            bestModel.dotsToDisplay=bestCandidateSet
            bestPercentage = percentageActual
    return bestCandidateSet, antiSetOfBestSet, bestModel

class CarPosition:

    def __init__(self, offset, isRight, isStandingToLine, angle):
        self.offset = offset
        self.isRight = isRight
        self.isStandingToLine = isStandingToLine
        self.angle = angle

    def attr2message(self):
        ret = [self.offset, self.isRight, self.isStandingToLine, self.angle]
        return Int16MultiArray(data = ret)

    def attr2string(self):
        sep = "/"
        return "" + str(self.offset) + sep + str(self.isRight) + sep + \
            str(self.isStandingToLine) + sep + str(self.angle)



def callback(data):
    global timestamp
    global carPositionPublisher

    #rospy.Publisher("line detection",Image, queue_size=20).publish(data.data)
    # rospy.loginfo("I heard and published type: data: {}, be {}, rows {}".format(data.data[0], data.encoding, data.height))
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)
    rgb_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
    gray_img = cv2.cvtColor(rgb_img, cv2.COLOR_RGB2GRAY)
    _, thresh = cv2.threshold(gray_img, 220, 255, cv2.THRESH_BINARY)
    whiteDots = np.argwhere(thresh > 0)
    whiteDots = whiteDots[:,::-1]

    # canvas=np.zeros((480,640))
    # canvas[whiteDots[:,1],whiteDots[:,0]]=255
    # plt.imshow(canvas,cmap='gray')
    # plt.show()

    # if (time.time() - timestamp) >= 5:
    #     plt.figure()
    #     plt.imshow(thresh, cmap='gray')
    #     plt.show()
    #     timestamp = time.time()

    _,_,lineModel = ransac(whiteDots)
    lineCenter = lineModel.antiFuncValue(480)
    #Positive Offset -> line is left of car; negatie Offst -> line is right of car
    offset = 320 - lineCenter
    carIsRight = offset>0


    #What Y Position does the intersection of the image middle with the measured line have?
    yOfIntersection = lineModel.funcValue(320)
    #This intersection can be above or below the image bottom
    #This tells us, wether the car is angled towards (intersection is above image bottom) or away from the line (intersection below)
    intersecctionIsAbove = yOfIntersection<480


    hypothenuse = math.sqrt(offset**2 + (yOfIntersection-480)**2)
    angleCarLine = math.asin(math.sin(offset/hypothenuse))

    carPos = CarPosition(offset, carIsRight, intersecctionIsAbove, angleCarLine)

    toPublish = carPos.attr2string()
    carPositionPublisher.publish(toPublish)
    rospy.loginfo("I published: {}".format(toPublish))

    # print(offset)
    # plt.figure()
    # plt.imshow(thresh, cmap='gray')
    # plt.show()

    # rospy.loginfo("I heard and published type: data: {}".format(cv_image))


def listener():
    global carPositionPublisher
    rospy.init_node("listener", anonymous=True)

    carPositionPublisher = rospy.Publisher("CarPosition", String, queue_size=10)
    rospy.Subscriber("/camera/color/image_raw", Image, callback)
    rospy.spin()

if __name__=='__main__':
    listener()
