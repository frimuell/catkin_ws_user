#drive_one_line

#!/usr/bin/env python
import rospy
from math import sqrt
from math import radians
import math
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Int16MultiArray
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt8
from std_msgs.msg import Int16
import time
from sensor_msgs.msg import LaserScan

def l2norm(vect):
    return math.sqrt(vect[0]**2 + vect[1]**2)

class OvalTrack:
    def __init__(self, center1, center2, radius):
        self.cTop = center1
        self.cBot = center2
        if (center2[0]<center1[0]):
            self.cTop = center1
            self.cBot = center2
        self.r = radius
        self.straightLineLength = self.cBot[0] - self.cTop[0]
        self.curvedLineLength = math.pi * self.r
        self.sectorStarts = [[self.cTop[0],self.cTop[1]+self.r],
                             [self.cTop[0],self.cTop[1]-self.r],
                             [self.cBot[0],self.cBot[1]-self.r],
                             [self.cBot[0],self.cBot[1]+self.r]]

    def getSector(self, point):
        x, y= point
        #Top Sector: return 0
        #Mid Left Sector: return 1
        #Bottom Sector: return 2
        #Mid Right Sector: return 3

        if y==self.cTop[1] and self.cTop[0]<=x and x<=self.cBot[0]:
            #Problematic case with multiple solutions
            #We Return the point with the smallest Y in this case - they are all in sector 1
            return 1
        elif x<=self.cTop[0]:
            return 0
        elif self.cTop[0]<x and x<self.cBot[0] and y<self.cTop[1]:
            return 1
        elif self.cBot[0]<=x:
            return 2
        elif self.cTop[0]<x and x<self.cBot[0] and self.cTop[1]<y:
            return 3
        else:
            return None

    def closestPointMeters(self, pointInMeters):
        oneMeterInPixels = 100
        closestPointInPixels = self.closestPointPixels([pointInMeters[0]*oneMeterInPixels,pointInMeters[1]*oneMeterInPixels])
        return [closestPointInPixels[0]/oneMeterInPixels,closestPointInPixels[1]/oneMeterInPixels]

    def closestPointPixels(self, point):
        x, y= point
        sector = self.getSector(point)
        if sector==0:
            vecFromCenterToPoint = [x-self.cTop[0],y-self.cTop[1]]
            vecLen = l2norm(vecFromCenterToPoint)
            return [self.cTop[0] + self.r*vecFromCenterToPoint[0]/vecLen, self.cTop[1] + self.r*vecFromCenterToPoint[1]/vecLen]
        elif sector==1:
            return [x,self.cTop[1]-self.r]
        elif sector==2:
            vecFromCenterToPoint = [x-self.cBot[0],y-self.cBot[1]]
            vecLen = l2norm(vecFromCenterToPoint)
            return [self.cBot[0] + self.r*vecFromCenterToPoint[0]/vecLen, self.cBot[1] + self.r*vecFromCenterToPoint[1]/vecLen]
        elif sector==3:
            return [x,self.cTop[1]+self.r]
        else:
            return None

    def distancesToSectorBorders(self, pointOnLine):
        x, y= pointOnLine

        sector = self.getSector(pointOnLine)
        if sector ==0:
            cosinus = (y-self.cTop[1])/self.r
            #This returns a distance in radians on a unit cirle
            #Multiply by r to get actual distance
            z = math.acos(cosinus)*self.r

            return z, self.curvedLineLength-z, sector
        elif sector ==1:
            z = self.cBot[0] - x
            return self.straightLineLength -z, z, sector
        if sector ==2:
            cosinus = (self.cBot[1]-y)/self.r
            #This returns a distance in radians on a unit cirle
            #Multiply by r to get actual distance
            z = math.acos(cosinus)*self.r

            return z, self.curvedLineLength-z, sector
        elif sector ==3:
            z = x - self.cTop[0]
            return self.straightLineLength -z, z, sector

    def phantomDriveMeters(self, pointOnLineInMeters, distanceInMeters):
        oneMeterInPixels = 100
        returnPointInPixels = self.phantomDrivePixels([pointOnLineInMeters[0]*oneMeterInPixels,pointOnLineInMeters[1]*oneMeterInPixels],distanceInMeters*oneMeterInPixels)
        return [returnPointInPixels[0]/oneMeterInPixels,returnPointInPixels[1]/oneMeterInPixels]

    def phantomDrivePixels(self, pointOnLineInPixels, distanceInPixels):
        secLengths = [self.curvedLineLength,self.straightLineLength,self.curvedLineLength,self.straightLineLength]

        toStart, toFinish, currentSector = self.distancesToSectorBorders(pointOnLineInPixels)
        distFromStart = distanceInPixels + toStart
        #Now we have the distance from the starting point of the currentSector
        while(distFromStart > secLengths[currentSector]):
            distFromStart -= secLengths[currentSector]
            currentSector = (currentSector+1)%4

        #Now we have a distance from the starting point of the currentSector, that is not longer than the current sector
        p = self.sectorStarts[currentSector]
        if currentSector == 1:
            return [p[0]+distFromStart,p[1]]
        elif currentSector == 3:
            return [p[0]-distFromStart,p[1]]
        elif currentSector == 0:
            arcus = distFromStart*math.pi/self.curvedLineLength
            x = self.cTop[0]-(math.sin(arcus)*self.r)
            y = self.cTop[1]+(math.cos(arcus)*self.r)
            return [x,y]
        elif currentSector == 2:
            arcus = distFromStart*math.pi/self.curvedLineLength
            x = self.cBot[0]+(math.sin(arcus)*self.r)
            y = self.cBot[1]-(math.cos(arcus)*self.r)
            return [x,y]
        else:
            return None

#Absolute Center of all Ovals
centerOfAll = [300,214.5]

#Inner Oval
distTopBotInner = 450.0
distLeftRightInner = 241.0
rInner = distLeftRightInner/2.0

#Outer Oval
ditTopBotOuter = 578.0
distLeftRightOuter = 369.0
rOuter = distLeftRightOuter/2.0

#This should be the same for inner and outer, so we only calculate it once
distanceOfCenters = distTopBotInner-2.0*rInner
centerTop = [centerOfAll[0]-distanceOfCenters/2.0,centerOfAll[1]]
centerBottom = [centerOfAll[0]+distanceOfCenters/2.0,centerOfAll[1]]

#25% and 75% between inner and outer line s
r25 = rInner + (rOuter-rInner)*0.25
r75 = rInner + (rOuter-rInner)*0.75

#The two Oval Tracks we need for task 1
ovalTrack25 = OvalTrack(centerTop, centerBottom, r25)
ovalTrack75 = OvalTrack(centerTop, centerBottom, r75)

#From the point of view of the camera's vector [1,0,0]
def getAngleToTarget(carPoint, targetPoint):
    y, x = targetPoint[1]-carPoint[1], targetPoint[0]-carPoint[0]
    return math.atan2(y, x)

#From the point of view of the car's vetcor [1,0,0]
def getCarOrientationAngle(z, w):
    return 2.0*math.acos(w) * (z/abs(z))


lastAbsAngle = 0.0
def pdController(actualAngle, p, d):
    global lastAbsAngle

    absAngle = abs(actualAngle)

    controlValue = math.copysign(1.0, actualAngle) * (absAngle*p + (lastAbsAngle-absAngle)*d)
    lastAbsAngle = absAngle


    rawSteeringAngle = 90.0 + (-90.0*(controlValue/math.pi))
    #print(str(actualAngle) + ";" + str(controlValue) + ";" + str(rawSteeringAngle))


    cappedSteeringAngle = max(min(rawSteeringAngle, 179.0), 0.0)
    return int(cappedSteeringAngle)

#Modulo to get an angle in the interval (-pi,+pi]. This might be too complicated?
def myModulo(angleBetweenMinusPiAndPi):
    return -((-angleBetweenMinusPiAndPi + math.pi)%(2*math.pi)) + math.pi

import math
import numpy as np
import matplotlib.pyplot as plt

#obstacle is in world coordinates, convert to car coordinates
def fromWorldToCar(obstacle, carPos, carOri):
    xO, yO = obstacle
    xC, yC = carPos
    cos = math.cos(carOri)
    sin = math.sin(carOri)
    newX = xO*((1- sin**2)/cos) + yO*sin -yC*sin + xC*((sin**2 -1)/cos)
    newY = -xO*sin + yO*cos - yC*cos + xC*sin
    return [newX, newY]

#sin(a)=gk/hyp -> sin(a)*hyp=gk
#cos(a)=ak/hyp -> cos(a)*hyp=ak
def getScatterPlot(lidarInput):
    temp = np.empty((len(lidarInput),2),dtype=float)

    for i in range(0,len(temp)):
        if lidarInput[i] == float('inf'):
            temp[i][0]=0
            temp[i][1]=0
        else:
            temp[i][0]=lidarInput[i]*math.cos(math.radians(i))
            temp[i][1]=lidarInput[i]*math.sin(math.radians(i))
    
    #Remove zeros (or infinities - this needs to be adjusted)
    temp = temp[np.logical_or(temp[:,0]!=0, temp[:,1]!=0)]
    
    return temp

def isAnObstacleNearPoint(data, point, distanceThreshold, densityThreshold):
    diff = data - point
    diff = np.linalg.norm(diff, axis=1)
    args = np.argwhere(diff<distanceThreshold)
    return len(args)>=densityThreshold


tracks = [ovalTrack25, ovalTrack75]
currentTrack = 0
def callbackLaserScan(data):
    if lastCarPoint is None:
        return None
    
    global currentTrack
    global lastLookAheadPoint
    lidarDots = getScatterPlot(data.ranges)
    #plt.scatter(lidarDots[:,0], lidarDots[:,1],0.1, color="black")
    #plt.savefig("testimg.png")
    lookAheadInCarCoords = fromWorldToCar(lastLookAheadPoint, carPos=lastCarPoint, carOri=lastCarOrientation)
    
    if isAnObstacleNearPoint(lidarDots, lookAheadInCarCoords, distanceThreshold=0.15, densityThreshold=4):
        #look if we are clear to switch, otherwise break
        #TODO
        
        #switch lines
        currentTrack = 1 - currentTrack
        print("SWITCH TO TRACK " + str(currentTrack))
        
    

lastCarPoint = None
lastCarOrientation = None
lastLookAheadPoint = None
def callback(data):
    global timestamp
    global lastCarPoint
    global lastCarOrientation
    global lastLookAheadPoint
    
    track = tracks[currentTrack]
    distance = 1.0

    pos = data.pose.pose.position
    x,y = pos.x, pos.y
    
    lastCarPoint = [x,y]

    closestPoint = track.closestPointMeters(lastCarPoint)

    lastLookAheadPoint = track.phantomDriveMeters(pointOnLineInMeters=closestPoint, distanceInMeters=distance)

    orientation = data.pose.pose.orientation
    z,w = orientation.z, orientation.w

    lastCarOrientation = getCarOrientationAngle(z,w)
    b = getAngleToTarget(carPoint=[x,y], targetPoint=lastLookAheadPoint)

    #Positive angle means the target point is left of the car. Negative means its right of it
    angleBetweenCarAndTarget = myModulo(b -lastCarOrientation)

    if (time.time()-timestamp) >= 0.2:
        timestamp = time.time()


        angleToPub = pdController(actualAngle=angleBetweenCarAndTarget, p=3.5, d=0.4)
        pub_steering.publish(angleToPub)
        #print(str(closestPoint[0]) + ";" + str(closestPoint[1]))
        #print(str(a) + ";" + str(b) + ";" + str(angleToPub))
        #print("angle: {} angleToPub: {}".format(angleBetweenCarAndTarget, angleToPub))

        pub_speed.publish(speed_rpm)




epsilon = 0.05
speed_rpm = 300
angle_straight = 90
angle_left = 20
angle_right = 20

callbackCnt = 0
timestamp = time.time()

offsetAvg = 0
cirleAvg = 0
offsetNum = 0
angleAvg = 0
last_odom = None
is_active = False

def KP(num):
    return 1.2* num

def KD(num):
    return -0.5 * num

def callbackOdom(msg):
    global last_odom
    last_odom = msg

def callbackDriveOnLine(data):
    global callbackCnt
    global timestamp
    global offsetAvg
    global circleAvg
    global offsetNum

    if "None" in data.data:
        return

    #radius = math.log2(int(data.data))

    #circleAvg
    offsetAvg += int(data.data)
    offsetNum += 1
    angle = angle_straight

    if (time.time()-timestamp) >= 0.5:
        off = offsetAvg/offsetNum
        angle -= KP(off)
        print("angle: {}".format(angle))
        pub_steering.publish(abs(angle))

        timestamp = time.time()

        offsetAvg=0
        offsetNum=0

        newSpeed = speed_rpm  - 0.4*abs(off)
        print("New Speed {}".format(newSpeed))
        pub_speed.publish(newSpeed)
    else:
        callbackCnt += 1



rospy.init_node("line", anonymous=True)
#rospy.Subscriber("/Offset", String, callbackDriveOnLine)

#sub_odom = rospy.Subscriber("odom", Odometry, callbackOdom, queue_size=10)

pub_back_left = rospy.Publisher(
    "simple_drive_control/backward_left",
    Float32,
    queue_size=10)
pub_back_right = rospy.Publisher(
    "simple_drive_control/backward_right",
    Float32,
    queue_size=10)
pub_back = rospy.Publisher(
    "simple_drive_control/backward",
    Float32,
    queue_size=10)
pub_forward = rospy.Publisher(
    "simple_drive_control/forward",
    Float32,
    queue_size=10)
pub_speed = rospy.Publisher(
    "manual_control/speed",
     Int16,
     queue_size=10)
pub_steering = rospy.Publisher(
    "steering",
    UInt8,
    queue_size=10)
pub_stop_start = rospy.Publisher(
    "manual_control/stop_start",
    Int16,
    queue_size=100)
pub_info = rospy.Publisher(
    "simple_drive_control/info",
    String,
    queue_size=10)
rospy.Subscriber("/localization/odom/12", Odometry, callback, queue_size=10)
rospy.Subscriber("/scan", LaserScan, callbackLaserScan)


rospy.spin()
