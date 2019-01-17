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



# from line_detection import CarPosition

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

    def closestPointMeters(self, pointInMeters):
        oneMeterInPixels = 100
        closestPointInPixels = self.closestPointPixels([pointInMeters[0]*oneMeterInPixels,pointInMeters[1]*oneMeterInPixels])
        return [closestPointInPixels[0]/oneMeterInPixels,closestPointInPixels[1]/oneMeterInPixels]

    def closestPointPixels(self, point):
        x, y= point
        if x<self.cTop[0]:
            vecFromCenterToPoint = [x-self.cTop[0],y-self.cTop[1]]
            vecLen = l2norm(vecFromCenterToPoint)
            return [self.cTop[0] + self.r*vecFromCenterToPoint[0]/vecLen, self.cTop[1] + self.r*vecFromCenterToPoint[1]/vecLen]
        elif self.cTop[0]<x and x<self.cBot[0] and y<self.cTop[1]:
            return [x,self.cTop[1]-self.r]
        elif self.cTop[0]<x and x<self.cBot[0] and self.cTop[1]<y:
            return [x,self.cTop[1]+self.r]
        elif self.cBot[0]<x:
            vecFromCenterToPoint = [x-self.cBot[0],y-self.cBot[1]]
            vecLen = l2norm(vecFromCenterToPoint)
            return [self.cBot[0] + self.r*vecFromCenterToPoint[0]/vecLen, self.cBot[1] + self.r*vecFromCenterToPoint[1]/vecLen]
        else:
            #Problematic case with multiple solutions. We Return the point with the smallest Y in this case
            return [x,self.cTop[1]-self.r]

distTopBot = 450.0
distLeftRight = 241.0
centerOfAll = [300,214.5]

r = distLeftRight/2.0
distanceOfCenters = distTopBot-2.0*r
centerTop = [centerOfAll[0]-distanceOfCenters/2.0,centerOfAll[1]]
centerBottom = [centerOfAll[0]+distanceOfCenters/2.0,centerOfAll[1]]

ovalTrack = OvalTrack(centerTop, centerBottom, r)

def callbackDistToOval(data):
    pos = data.pose.pose.position
    x,y = pos.x, pos.y
    closestPoint = ovalTrack.closestPointMeters([x,y])
    distanceInMeters = l2norm([x-closestPoint[0],y-closestPoint[1]])
    print(distanceInMeters)

epsilon = 0.05
speed_rpm = 180
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
    return 0.35 * num

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
rospy.Subscriber("/localization/odom/4", Odometry, callbackDistToOval, queue_size=10)

rospy.spin()
