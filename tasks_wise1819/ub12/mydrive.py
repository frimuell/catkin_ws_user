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


import rospy
import sys

import time
import numpy as np
from std_msgs.msg import Int16, UInt8, Float32
import signal
import matplotlib.pyplot as plt

# self.aimed_mps = meters_per_second # input value
#aimed_tps = meters_per_second / ((5/9)/100) # 5/9 = 1 tick = 5/9cm

current_speed = 0 # in Int16 values for topic /manual_control/speed Int16
current_tps = 0
ticks = 0 # 1m = ca 180 ticks
time_sum = 0 # runtime in seconds

K_P = 0.7 # to be adjusted by trial and error
# small = slower oscillation and more time for I and D to work
# <1 is advised

K_I = 0.075 # to be adjusted by trial and error

K_D = 0.9 # to be adjusted by trial and error

time_sum = 0 # runtime in seconds
error_sum = 0

error_prev = 0
ticks_prev = 0


def callback_ticks(data):
    global ticks
    ticks += int(data.data)
# use the counted ticks of the last second to compute velocity and execute PID control
def pid_controller(meters_per_second,time_elapsed):
    global ticks_prev
    global error_prev
    global error_sum
    global time_sum
    global current_tps
    global current_speed

    aimed_tps = meters_per_second / ((5.0/9.0)/100.0) # 5/9 = 1 tick = 5/9cm


    current_tps  = (ticks - ticks_prev)/time_elapsed

    print("aimed_tps", aimed_tps,  "current_tps", current_tps)


    error = aimed_tps - current_tps
    #error_sum += error * time_sum
    #deriv = error - error_prev # discrete derivative (delta t = 1 second)

    #speed_increase = K_P * error + K_D * deriv + K_I * error_sum  # P + I + D
    speed_increase = K_P * error
    print("speed: ", current_speed, "speed increase: ", speed_increase)
    print("ticks: ", ticks, "\n")
    current_speed += speed_increase
    #current_speed = aimed_tps

    speed_pub.publish(np.int16(current_speed))

    #error_prev = error
    ticks_prev = ticks
    #time_sum += 1

    #time.sleep(1 - (time.time() % 1))

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
angleInegral = 0.0
def pdController(actualAngle, p, d, i=0.0123):
    global lastAbsAngle
    global angleInegral

    #angleInegral += actualAngle
    #print(angleInegral)

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

    if isAnObstacleNearPoint(lidarDots, lookAheadInCarCoords, distanceThreshold=0.15, densityThreshold=3):
        #look if we are clear to switch, otherwise break
        #TODO

        #switch lines
        currentTrack = 1 - currentTrack
        print("SWITCH TO TRACK " + str(currentTrack))

def movePoint(point, angle, dist):
    return [point[0] + math.cos(angle)*dist, point[1] + math.sin(angle)*dist]

lastCarPoint = None
lastCarOrientation = None
lastLookAheadPoint = None
distance = 0.8

big_elapse = 0.0
def callback(data):
    global timestamp
    global lastCarPoint
    global lastCarOrientation
    global lastLookAheadPoint
    global big_elapse

    track = tracks[currentTrack]


    pos = data.pose.pose.position
    x,y = pos.x, pos.y

    orientation = data.pose.pose.orientation
    z,w = orientation.z, orientation.w

    #print("x: "+ str(x)+" y: "+ str(y)+" z: "+ str(z)+" w: "+ str(w))

    lastCarOrientation = getCarOrientationAngle(z,w)

    #Move Point from back of car (where the "gps" puts it) to front of car (where the lidar is)
    #Distance between these two points is about 25cm
    lastCarPoint = movePoint(point=[x,y], angle=lastCarOrientation, dist=0.2)

    #Bias the car 20 cm to the left
    lastCarPoint = movePoint(point=lastCarPoint, angle=myModulo(lastCarOrientation-math.pi/2), dist=0.2)

    #print(str([x,y])+";"+str(lastCarPoint))
    #lastCarPoint = [x,y]
    closestPoint = track.closestPointMeters(lastCarPoint)

    lastLookAheadPoint = track.phantomDriveMeters(pointOnLineInMeters=closestPoint, distanceInMeters=distance)
    b = getAngleToTarget(carPoint=lastCarPoint, targetPoint=lastLookAheadPoint)

    #Positive angle means the target point is left of the car. Negative means its right of it
    angleBetweenCarAndTarget = myModulo(b -lastCarOrientation)
    #elapsed = time.time()-timestamp
    #big_elapse += elapsed
    if (time.time()-timestamp) >= 0.2:
        timestamp = time.time()

        angleToPub = pdController(actualAngle=angleBetweenCarAndTarget, p=3.0, d=0.7)

        pub_steering.publish(angleToPub)


        #print(str(closestPoint[0]) + ";" + str(closestPoint[1]))
        #print(str(a) + ";" + str(b) + ";" + str(angleToPub))
        #print("angle: {} angleToPub: {}".format(angleBetweenCarAndTarget, angleToPub))
        factor = 2.0
        bottomCap = 0.6
        #if (big_elapse>1.0):
        #    pid_controller(0.6, big_elapse)
        #    big_elapse = 0.0
        speed = max(speed_rpm*bottomCap, speed_rpm - speed_rpm*factor*(abs(angleBetweenCarAndTarget)/math.pi))
        #print(speed)
        pub_speed.publish(speed)




epsilon = 0.05
speed_rpm = 400
angle_straight = 90
angle_left = 20
angle_right = 20

callbackCnt = 0
timestamp = time.time()


last_odom = None

def callbackOdom(msg):
    global last_odom
    last_odom = msg


rospy.init_node("line", anonymous=True)

#sub_odom = rospy.Subscriber("odom", Odometry, callbackOdom, queue_size=10)
ticks_sub = rospy.Subscriber("/ticks", UInt8, callback_ticks, queue_size=20)
speed_pub = rospy.Publisher("/manual_control/speed", Int16, queue_size=20)

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
rospy.Subscriber("/localization/odom/11", Odometry, callback, queue_size=10)
rospy.Subscriber("/scan", LaserScan, callbackLaserScan)

def myShutdown():
    pub_speed.publish(0)

rospy.on_shutdown(myShutdown)

rospy.spin()

#rospy.init_node('pid_velocity', anonymous=True)

# def rescue(*_):
#     print(pid_vel.output)
#     rospy.signal_shutdown("")
#
# signal.signal(signal.SIGINT, rescue)

#rospy.spin()
