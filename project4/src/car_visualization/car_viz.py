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

# Create a figure with 2x2 subplots
fig, axes = plt.subplots(1, 2, figsize=(8, 4))
axes = axes.flatten()

# Car Street Environment - RRT
axes[0].axis('equal')
axes[0].set_xlim(-10, 10)
axes[0].set_ylim(-10, 10)

positions = [(-11, -6, 8, 10), 
             (2, -6, 9, 10), 
             (-7, 8, 14, 3)]

for position in positions:
    plot_rectangle(axes[0], position)

data = np.loadtxt('car_RRT.txt')
axes[0].plot(data[:, 0], data[:, 1], 'o-', markersize=2)
axes[0].plot(-8, -8, 'bs', markersize=2) # Plot the start (-8, -8)
axes[0].plot(9, 9, 'rs', markersize=2) # Plot the goal (9, 9)
axes[0].set_xlabel('South Street')
axes[0].set_ylabel('West Street')
axes[0].set_title('Car Street Environment - RRT')

# Car Street Environment - KPIECE
axes[1].axis('equal')
axes[1].set_xlim(-10, 10)
axes[1].set_ylim(-10, 10)

positions = [(-11, -6, 8, 10), 
             (2, -6, 9, 10), 
             (-7, 8, 14, 3)]

for position in positions:
    plot_rectangle(axes[1], position)

data = np.loadtxt('car_RRT.txt')
axes[1].plot(data[:, 0], data[:, 1], 'o-', markersize=2)
axes[1].plot(-8, -8, 'bs', markersize=2) # Plot the start (-8, -8)
axes[1].plot(9, 9, 'rs', markersize=2) # Plot the goal (9, 9)
axes[1].set_xlabel('South Street')
axes[1].set_ylabel('West Street')
axes[1].set_title('Car Street Environment - KPIECE')

# Adjust spacing between subplots
plt.tight_layout()

plt.show()
