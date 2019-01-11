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
import matplotlib.patches as patches

carPositionPublisher = None
timestamp = time.time()
timestamp2 = time.time()

def histogramm(img):
    whitePts = []
    for i in range(len(img[0])):
        column = img[:,i]
        whitePts.append(np.count_nonzero(column == 255))

    return whitePts

def getWhitePtsOfWindow(img, win, shape): #shape = (hight * width)
    firstRow = win[2][1]
    intervall = [win[0][0],win[1][0]]
    res = 0
    avgBlackPtsLeft = 0
    avgBlackPtsRight = 0
    for i in range(shape[1]):
        #print(img)
        row = img[int(firstRow) + i]

        winRow = row[int(win[1][0]):int(win[2][0])]

        wasWhite = False
        for j in range(len(winRow)):
            if winRow[j] != 255:
                if(wasWhite == False):
                    avgBlackPtsLeft += 1
                else:
                    avgBlackPtsRight += 1
            if winRow[j] == 255:
                wasWhite = True
        #print(winRow)
        numOfWhitePts = np.count_nonzero(winRow == 255)
        res += numOfWhitePts
    avgBlackPtsLeft = avgBlackPtsLeft/shape[1]
    avgBlackPtsRight = avgBlackPtsRight/shape[1]

    return (res, avgBlackPtsLeft, avgBlackPtsRight)

def moveWin2Left(win, num):
    resShape = win
    resShape[0][0] = win[0][0] - num
    resShape[1][0] = win[1][0] - num
    resShape[2][0] = win[2][0] - num
    resShape[3][0] = win[3][0] - num
    return resShape

def moveWin2Right(win, num):
    resShape = win
    resShape[0][0] = win[0][0] + num
    resShape[1][0] = win[1][0] + num
    resShape[2][0] = win[2][0] + num
    resShape[3][0] = win[3][0] + num
    return resShape

def windowDetection(img):
    #inputImage = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    inputImage = img

    src2 = np.float32([[280,270],[360,270],[460,435],[180,435]])
    dst2 = np.float32([[192,0],[448,0],[448, 480],[192,480]])
    matrix = cv2.getPerspectiveTransform(src2,dst2)
    resImg = cv2.warpPerspective(inputImage, matrix, (640,480))

    whitePts = histogramm(resImg[400:481])


    leftLine = []
    foundWhitePt = False
    for i in range(len(whitePts)):
        if whitePts[i] != 0:
            leftLine.append(i)
            foundWhitePt = True
        else:
            if foundWhitePt:
                break;
    if len(leftLine) > 0:
        first = leftLine[0]
        last = leftLine[len(leftLine)-1]

    window = (120,30)
    maxWhitePts = np.argmax(whitePts)
    startHight = 480

    firstShape = [[maxWhitePts-(window[0]/2),startHight],
                  [maxWhitePts-(window[0]/2),startHight-window[1]],
                  [maxWhitePts+window[0]-(window[0]/2),startHight-window[1]],
                  [maxWhitePts+window[0]-(window[0]/2),startHight]]

    all_x = []
    all_y = []
    currentShape = firstShape
    fig,ax = plt.subplots(1)
    midPointList = []
    for i in range(int(startHight/window[1]) - 3):
        nextShape = currentShape
        nextShape[0][1] = currentShape[0][1] - window[1]
        nextShape[1][1] = currentShape[1][1] - window[1]
        nextShape[2][1] = currentShape[2][1] - window[1]
        nextShape[3][1] = currentShape[3][1] - window[1]

        nextShape = np.array(nextShape)

        temp = getWhitePtsOfWindow(resImg, nextShape, window)
        curWhitPts = temp[0]


        if(temp[1] > temp[2]):
            diff = (temp[1] - temp[2])/2
            nextShape = moveWin2Right(nextShape, diff)
        elif(temp[2] > temp[1]):
            diff = (temp[2] - temp[1])/2
            nextShape = moveWin2Left(nextShape, diff)

        temp = getWhitePtsOfWindow(resImg, nextShape, window)
        if temp[0] == 0:
            continue
        rect = patches.Rectangle((nextShape[1][0],
                                  nextShape[1][1]),
                                  window[0],window[1],
                                  linewidth=1,edgecolor='r',facecolor='none')

        ax.add_patch(rect)
        #np.concatenate((midPointList,np.array([[(nextShape[2][0] + nextShape[1][0])/2,nextShape[3][1]]])),axis=0)
        midPointList.append([(nextShape[2][0] + nextShape[1][0])/2,nextShape[3][1]])
        midPointList.append([(nextShape[3][0] + nextShape[0][0])/2,nextShape[3][1]])
        #np.concatenate((midPointList,np.array([[(nextShape[3][0] + nextShape[0][0])/2,nextShape[3][1]]])),axis=0)


        currentShape = nextShape
        radius = 0
    if len(midPointList) > 2:
        midPointList2 = np.array(midPointList)
        x = midPointList2[:,0]
        y = midPointList2[:,1]

        x_lineCenter = x[len(x)//2]
        polynom = np.polyfit(x, y, 3)
        #polynom = np.array([4,3,2,1])

        deriv_poly = [polynom[len(polynom)-1-i] * i for i in range(len(polynom)-1, -1, -1)]
        deriv_poly = np.array(deriv_poly)
        deriv_poly = deriv_poly[:-1]
        #print(deriv_poly)
        res = 0
        exp = 2
        for coeff in deriv_poly:
            res += coeff*(x_lineCenter**exp)
            exp -= 1
        zaehler = (1 + res**2)**(3/2)

        deriv_poly = deriv_poly**2
        res = 0
        res += deriv_poly[0]*(x_lineCenter**4)
        res += deriv_poly[1]*(x_lineCenter)
        res += deriv_poly[2]

        nenner = res
        radius = zaehler/nenner
        print(radius)

    # global timestamp2
    # if time.time()-timestamp2>= 10:
    #     plt.imshow(resImg,cmap='gray')
    #     plt.show()
    #     timestamp2=time.time()
    plt.imshow(resImg,cmap='gray')
    plt.show()
    # timestamp2=time.time()
    #fig.savefig("windowImg.png")


    firstShape = np.array(firstShape)
    #print("First Window: {}".format(firstShape))
    x = firstShape[:,0]
    y = firstShape[:,1]

    return radius
    # plt.figure()
    # plt.imshow(resImg,cmap='gray')
    # plt.show()
    #
    # x2 = list(range(len(whitePts)))
    # plt.plot(x2, whitePts)
    # plt.plot(x,y)
    # plt.show()
    # print(getWhitePtsOfWindow(resImg, firstShape, window))



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

    rad = windowDetection(thresh)

    carPositionPublisher.publish(str(rad)+":"+str(offset))

def listener():
    global carPositionPublisher
    rospy.init_node("listener", anonymous=True)

    carPositionPublisher = rospy.Publisher("Offset", String, queue_size=10)
    rospy.Subscriber("/camera/color/image_raw", Image, callback)
    rospy.spin()

if __name__=='__main__':
    listener()
