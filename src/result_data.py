#!/usr/bin/env python
import rospy

from std_msgs.msg import Float32

lista = []
def callback(data):
	rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
	lista.append(data.data)
	print (lista)

def listener():

	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber('/data/ToPy', Float32, callback)

	rospy.spin()

if __name__ == '__main__':
	listener()