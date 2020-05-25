#!/usr/bin/env python
import rospy
import matplotlib.pyplot as plt
import numpy as np

from std_msgs.msg import Float32
from std_msgs.msg import Int32

''' INSTRUCCIONES PARA GRAFICAR

Una vez realizada la simualcion deseada, publicar mensaje en el topic de la siguiente forma:
rostopic pub /data/PrintGraph std_msgs/Int32 1
Para una comoda seleccion del tipo de grafica se enviara un numero distinto en el mensaje segun proceda
1 --> KPH PFH
2 --> KPH FPHF
3 --> KPISS PFH
4 --> KPISS FPHF
5 --> SinKP PFH
6 --> SinKp FPFH

'''

valores = np.array([])

def callback(data):
	global valores

	rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
	valores = np.append(valores, data.data)

def callback_print(data):
	rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
	if data.data != 0:
		fig, axs = plt.subplots()
		plt.plot(range(valores.size),valores)
		axs.set_xlabel('Scan procesados')
		axs.set_ylabel('Numero de puntos con correspondencia')

		if data.data == 1:
			fig.suptitle('Correspondencias Harris y PFH', fontsize=16)

		elif data.data == 2:
			fig.suptitle('Correspondencias Harris y FPFH', fontsize=16)

		elif data.data == 3:
			fig.suptitle('Correspondencias ISS y PFH', fontsize=16)

		elif data.data == 4:
			fig.suptitle('Correspondencias ISS y FPFH', fontsize=16)

		elif data.data == 5:
			fig.suptitle('Correspondencias sin keypoints y PFH', fontsize=16)

		elif data.data == 6:
			fig.suptitle('Correspondencias sin keypoints y FPFH', fontsize=16)


		plt.axis([0, valores.size, 0, np.max(valores)+10])
		plt.show()

def listener():

	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber('/data/ToPy', Float32, callback)
	rospy.Subscriber('/data/PrintGraph', Int32, callback_print)

	rospy.spin()

if __name__ == '__main__':
	listener()