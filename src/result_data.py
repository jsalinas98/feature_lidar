#!/usr/bin/env python
import rospy
import matplotlib.pyplot as plt
import numpy as np

from std_msgs.msg import Float32
from std_msgs.msg import Int32

valores = np.array([])

def callback(data):
	global valores

	rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
	valores = np.append(valores, data.data)

def callback_print(data):
	rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
	if data.data != 0:
		print(valores)
		print(valores.size)
		plt.plot(range(valores.size),valores)
		plt.axis([0, valores.size, 0, 15])
		plt.show()

def listener():

	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber('/data/ToPy', Float32, callback)
	rospy.Subscriber('/data/PrintGraph', Int32, callback_print)

	rospy.spin()

if __name__ == '__main__':
	listener()