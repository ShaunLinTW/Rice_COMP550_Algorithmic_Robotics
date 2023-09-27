from mpl_toolkits.mplot3d import Axes3D
import numpy
import matplotlib.pyplot as plt

data = numpy.loadtxt('box_env1.txt')
fig = plt.figure()
ax = fig.add_subplot(projection='3d')
# ax = fig.add_subplot()
# print(data.shape)
ax.plot(data[:,0],data[:,1],data[:,2],'.-')
# ax.plot(data[:,0],data[:,1],'.-')
plt.show()