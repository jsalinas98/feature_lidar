import matplotlib.pyplot as plt
import numpy as np

#Scans procesados simulando la primera bag
x = np.arange(6)
y = np.array([1975, 1441, 1457, 1000, 432, 717])

fig, ax = plt.subplots()
ax.bar(x[0],y[0], color = 'b', label = 'KPH PFH')
ax.bar(x[1],y[1], color = 'g', label = 'KPH FPFH')
ax.bar(x[2],y[2], color = 'r', label = 'KPISS PFH')
ax.bar(x[3],y[3], color = 'm', label = 'KPISS FPFH')
ax.bar(x[4],y[4], color = 'c', label = 'Sin KP PFH')
ax.bar(x[5],y[5], color = 'y', label = 'Sin KP FPFH')

fig.suptitle('Comparacion numero de scans por feature', fontsize=16)

ax.set_ylabel('Numero de scans')

plt.legend()
plt.show()

#Scans proceados simulando la segunda bag
x = np.arange(6)
y = np.array([609, 648, 526, 505, 141, 198])

fig, ax = plt.subplots()
ax.bar(x[0],y[0], color = 'b', label = 'KPH PFH')
ax.bar(x[1],y[1], color = 'g', label = 'KPH FPFH')
ax.bar(x[2],y[2], color = 'r', label = 'KPISS PFH')
ax.bar(x[3],y[3], color = 'm', label = 'KPISS FPFH')
ax.bar(x[4],y[4], color = 'c', label = 'Sin KP PFH')
ax.bar(x[5],y[5], color = 'y', label = 'Sin KP FPFH')

fig.suptitle('Comparacion numero de scans por feature', fontsize=16)

ax.set_ylabel('Numero de scans')

plt.legend()
plt.show()

#Numero de KP medios BAG 1
x = np.arange(4)
y = np.array([410.04, 164.07, 108.25, 68.05])

fig, ax = plt.subplots()
ax.bar(x[0],y[0], color = 'b', label = 'KPH')
ax.bar(x[1],y[1], color = 'g', label = 'KPISS')
ax.bar(x[2],y[2], color = 'r', label = 'SIFT Z')
ax.bar(x[3],y[3], color = 'm', label = 'SIFT NE')

fig.suptitle('Comparacion numero de keypoints', fontsize=16)

ax.set_ylabel('Numero de keypoints')

plt.legend()
plt.show()

#Numero de KP medios BAG 2
x = np.arange(4)
y = np.array([317.03, 133.95, 378.95, 148.98])

fig, ax = plt.subplots()
ax.bar(x[0],y[0], color = 'b', label = 'KPH')
ax.bar(x[1],y[1], color = 'g', label = 'KPISS')
ax.bar(x[2],y[2], color = 'r', label = 'SIFT Z')
ax.bar(x[3],y[3], color = 'm', label = 'SIFT NE')

fig.suptitle('Comparacion numero de keypoints', fontsize=16)

ax.set_ylabel('Numero de keypoints')

plt.legend()
plt.show()

#Numero de correspondencia medio bag 1
x = np.arange(6)
y = np.array([9.15, 4.18, 5.71, 3.95, 54.29, 49.67])

fig, ax = plt.subplots()
ax.bar(x[0],y[0], color = 'b', label = 'KPH PFH')
ax.bar(x[1],y[1], color = 'g', label = 'KPH FPFH')
ax.bar(x[2],y[2], color = 'r', label = 'KPISS PFH')
ax.bar(x[3],y[3], color = 'm', label = 'KPISS FPFH')
ax.bar(x[4],y[4], color = 'c', label = 'Sin KP PFH')
ax.bar(x[5],y[5], color = 'y', label = 'Sin KP FPFH')

fig.suptitle('Comparacion numero de correspondencia medio', fontsize=16)

ax.set_ylabel('Numero de correspondencias')

plt.legend(loc='upper left')
plt.show()

#Numero de correspondencia medio bag 2
x = np.arange(6)
y = np.array([6.41, 4.12, 4.27, 4.03, 58.77, 49.31])

fig, ax = plt.subplots()
ax.bar(x[0],y[0], color = 'b', label = 'KPH PFH')
ax.bar(x[1],y[1], color = 'g', label = 'KPH FPFH')
ax.bar(x[2],y[2], color = 'r', label = 'KPISS PFH')
ax.bar(x[3],y[3], color = 'm', label = 'KPISS FPFH')
ax.bar(x[4],y[4], color = 'c', label = 'Sin KP PFH')
ax.bar(x[5],y[5], color = 'y', label = 'Sin KP FPFH')

fig.suptitle('Comparacion numero de correspondencia medio', fontsize=16)

ax.set_ylabel('Numero de correspondencias')

plt.legend(loc='upper left')
plt.show()

