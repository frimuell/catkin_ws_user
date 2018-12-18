#!/usr/bin/env python
import rospy
from math import sqrt
from math import radians
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Int16MultiArray
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt8
from std_msgs.msg import Int16
import time


# from line_detection import CarPosition
epsilon = 0.05
speed_rpm = 100
angle_straight = 90
angle_left = 20
angle_right = 20

callbackCnt = 0
timestamp = time.time()

offsetAvg = 0
angleAvg = 0
last_odom = None
is_active = False


def drive(distance, command, speed, angle):
    global is_active

    if command != "Forward":
        rospy.loginfo("%s: Running %s(%f)", rospy.get_caller_id(), command, distance)

    if distance <= 0:
        rospy.logerr(
            "%s: Error, distance argument has to be > 0! %f given",
            rospy.get_caller_id(),
            distance)
        return

    pub_info.publish("BUSY")
    if is_active:
        rospy.logwarn(
            "%s: Warning, another command is still active! Please wait and try again.",
            rospy.get_caller_id())
        return

    is_active = True

    # stop the car and set desired steering angle + speed
    pub_speed.publish(0)
    # pub_stop_start.publish(1)
    rospy.sleep(1)
    pub_steering.publish(angle)
    # pub_stop_start.publish(0)
    rospy.sleep(1)
    pub_speed.publish(speed)

    start_pos = last_odom.pose.pose.position
    current_distance = 0

    while not rospy.is_shutdown() and current_distance < (distance - epsilon):
        current_pos = last_odom.pose.pose.position
        current_distance = sqrt(
            (current_pos.x - start_pos.x)**2 + (current_pos.y - start_pos.y)**2)
        # rospy.loginfo("current distance = %f", current_distance)
        rospy.sleep(0.1)

    pub_speed.publish(0)
    is_active = False
    current_pos = last_odom.pose.pose.position
    current_distance = sqrt((current_pos.x - start_pos.x)** 2 + (current_pos.y - start_pos.y)**2)
    pub_info.publish("FINISHED")

    rospy.loginfo(
        "%s: Finished %s(%f)\nActual travelled distance = %f",
        rospy.get_caller_id(),
        command,
        distance,
        current_distance)

def callbackOdom(msg):
    global last_odom
    last_odom = msg

def callbackDriveOnLine(data):
    global callbackCnt
    niceDriveTowards = 10

    class stump:
         def __init__(self,input):
             temp = input.split('/')
             self.offset=int(temp[0])
             self.isRight= (temp[1] == "True")
             self.isStandingToLine= (temp[2] == "True")
             self.angle=float(temp[3])


    carPos = stump(data.data)
#    global offsetAvg
#    global angleAvg

    angle = angle_straight
    command = "Forward"

    global timestamp

    if(time.time()-timestamp) >= 1:
#        offsetAvg = offsetAvg / callbackCnt

        # Car is on the right side of the line
        if(carPos.isRight and not carPos.isStandingToLine):
            angle=int((- carPos.angle*(180/3.14)) + angle - niceDriveTowards)
            command = "Left"
#            if offsetAvg > 100:
#                drive(0.6, "Left", speed_rpm, angle_left)
#            elif offsetAvg > 50:
#                drive(0.2, "Left", speed_rpm, angle_left)

        # Car is on the left side of the line
        elif(not carPos.isRight and not carPos.isStandingToLine):
            angle=int((- carPos.angle*(180/3.14)) + angle + niceDriveTowards)
            command = "Right"
#            if offsetAvg > 100:
#                drive(0.6, "Left", speed_rpm, angle_right)
#            elif offsetAvg > 50:
#                drive(0.2, "Left", speed_rpm, angle_right)

        print("isRight: {}; isStandingToLine: {}; command: {};angle: {}".format(carPos.isRight, carPos.isStandingToLine, command, angle))
        timestamp = time.time()

#        offsetAvg = 0
        # print("Trying to drive: {}".format(command))
        pub_steering.publish(angle)
        pub_speed.publish(speed_rpm)
        # drive(0.5, command, speed_rpm, angle)

    else:
#        offsetAvg = offsetAvg + abs(data.offset)
#        angleAvg = angleAvg + data.angle
        callbackCnt += 1



rospy.init_node("line", anonymous=True)
rospy.Subscriber("/CarPosition", String, callbackDriveOnLine)

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

# pub_speed.publish(speed_rpm)
rospy.spin()
