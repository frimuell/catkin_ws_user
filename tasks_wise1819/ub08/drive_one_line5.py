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


epsilon = 0.05
speed_rpm = 200
angle_straight = 90
angle_left = 20
angle_right = 20

callbackCnt = 0
timestamp = time.time()

offsetAvg = 0
circleAvg = 0
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

    recData = data.data.split(':')
    radius = math.log(int(recData[0]),2)
    print("Radius: {}".format(data.data))

    #circleAvg
    offsetAvg += int(recData[1])
    offsetNum += 1
    angle = angle_straight

    if (time.time()-timestamp) >= 0.5:
        off = offsetAvg/offsetNum
        angle -= KP(0.5*off + 0.5*radius)
        print("angle: {}".format(angle))
        pub_steering.publish(abs(angle))

        timestamp = time.time()

        offsetAvg=0
        offsetNum=0

        newSpeed = speed_rpm
        if radius >= 18:
            newSpeed = speed_rpm + 100
        print("New Speed {}".format(newSpeed))
        pub_speed.publish(newSpeed)
    else:
        callbackCnt += 1



rospy.init_node("line", anonymous=True)
rospy.Subscriber("/Offset", String, callbackDriveOnLine)

sub_odom = rospy.Subscriber("odom", Odometry, callbackOdom, queue_size=10)

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

rospy.spin()
