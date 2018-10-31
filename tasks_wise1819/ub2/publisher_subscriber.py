import rospy
from std_msgs.msg import Float32
from std_msgs.msg import String

def callback(data):
	rospy.Publisher('assignment1_publisher_subscriber', String , queue_size=20) .publish("I heard " + str(data))
	rospy.loginfo('I heard and published %s:', data.data)

def listener():
	rospy.init_node('listener', anonymous=True)

	rospy.Subscriber('/yaw', Float32, callback)

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

if __name__ == '__main__':
	listener()

