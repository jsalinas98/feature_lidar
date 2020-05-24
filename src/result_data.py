#!/usr/bin/env python
import rospy
import matplotlib.pyplot as plt
import numpy as np

from std_msgs.msg import Float32

valores = np.array([])

def callback(data):
	rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
	np.append(valores, data.data)
	if(valores.size>20):
		plt.plot(np.arange(0,valores.size,1),valores)
		plt.show()

def listener():

	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber('/data/ToPy', Float32, callback)

	rospy.spin()

if __name__ == '__main__':
	listener()