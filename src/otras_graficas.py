import matplotlib.pyplot as plt
import numpy as np

x = np.arange(6)
y = np.array([1975, 1441, 1457, 1000, 432, 717])

fig, ax = plt.subplots()
ax.bar(x[0],y[0], color = 'b', label = 'KPH PFH')
ax.bar(x[1],y[1], color = 'g', label = 'KPH FPFH')
ax.bar(x[2],y[2], color = 'r', label = 'KPISS PFH')
ax.bar(x[3],y[3], color = 'm', label = 'KPISS FPFH')
ax.bar(x[4],y[4], color = 'c', label = 'Sin KP PFH')
ax.bar(x[5],y[5], color = 'y', label = 'Sin KP FPFH')

fig.suptitle('Compacion numero de scans por feature', fontsize=16)

ax.set_ylabel('Numero de scans en 260 s')

plt.legend()
plt.show()