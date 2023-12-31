'''
///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: Jason Zhang(jz118), Shaun Lin(hl116)
//////////////////////////////////////
'''

from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Polygon

# Function to plot rectangles
def plot_rectangle(ax, position):
    rect = Rectangle(position[:2], position[2], position[3], fill=False)
    ax.add_patch(rect)

# Create a figure with 1x3 subplots
fig, axes = plt.subplots(1, 3, figsize=(12, 4))
axes = axes.flatten()

# Car Planning using RRT
axes[0].axis('equal')
axes[0].set_xlim(-10, 10)
axes[0].set_ylim(-10, 10)

positions = [(-11, -6, 8, 10), 
             (3, -6, 8, 10), 
             (-7, 8, 14, 3),
             (-1, -6, 2, 10)]

for position in positions:
    plot_rectangle(axes[0], position)

data = np.loadtxt('car_RRT.txt')
axes[0].plot(data[:, 0], data[:, 1], 'o-', markersize=2)
axes[0].plot(-8, -8, 'bs', markersize=2) # Plot the start (-8, -8)
axes[0].plot(9, 9, 'rs', markersize=2) # Plot the goal (9, 9)
axes[0].set_xlabel('South Street')
axes[0].set_ylabel('West Street')
axes[0].set_title('Car Planning using RRT')
# RRT used 65754 states, and found solution in 60.05 seconds

# Car Planning using KPIECE
axes[1].axis('equal')
axes[1].set_xlim(-10, 10)
axes[1].set_ylim(-10, 10)

positions = [(-11, -6, 8, 10), 
             (3, -6, 8, 10), 
             (-7, 8, 14, 3),
             (-1, -6, 2, 10)]

for position in positions:
    plot_rectangle(axes[1], position)

data = np.loadtxt('car_KPIECE.txt')
axes[1].plot(data[:, 0], data[:, 1], 'o-', markersize=2)
axes[1].plot(-8, -8, 'bs', markersize=2) # Plot the start (-8, -8)
axes[1].plot(9, 9, 'rs', markersize=2) # Plot the goal (9, 9)
axes[1].set_xlabel('South Street')
axes[1].set_ylabel('West Street')
axes[1].set_title('Car Planning using KPIECE')
# KPIECE used 173097 states, and found solution in 60.05 seconds

# Car Planning using RG-RRT
axes[2].axis('equal')
axes[2].set_xlim(-10, 10)
axes[2].set_ylim(-10, 10)

positions = [(-11, -6, 8, 10),
                (3, -6, 8, 10),
                (-7, 8, 14, 3),
                (-1, -6, 2, 10)]

for position in positions:
    plot_rectangle(axes[2], position)

data = np.loadtxt('car_RG-RRT.txt')
axes[2].plot(data[:, 0], data[:, 1], 'o-', markersize=2)
axes[2].plot(-8, -8, 'bs', markersize=2) # Plot the start (-8, -8)
axes[2].plot(9, 9, 'rs', markersize=2) # Plot the goal (9, 9)
axes[2].set_xlabel('South Street')
axes[2].set_ylabel('West Street')
axes[2].set_title('Car Planning using RG-RRT')
# RG-RRT used 32253 states, and found solution in 60.03 seconds

# Adjust spacing between subplots
plt.tight_layout()

plt.show()
