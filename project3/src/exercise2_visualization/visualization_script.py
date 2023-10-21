'''
///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 3
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
fig, axes = plt.subplots(2, 2, figsize=(5.5, 5.5))
axes = axes.flatten()

# Point Robot in Environment 1
axes[0].axis('equal')
axes[0].set_xlim(-6, 6)
axes[0].set_ylim(-6, 6)

positions = [(-4, -4, 4.5, 4), 
             (2, -4, 4, 4), 
             (2, 0.2, 3, 3), 
             (-4, 1, 3.5, 4.5)]
for position in positions:
    plot_rectangle(axes[0], position)

data = np.loadtxt('point_env1.txt')
axes[0].plot(data[:, 0], data[:, 1], 'o-', markersize=2)
axes[0].plot(-5, -5, 'bo', markersize=2) # Plot the start (-5, -5)
axes[0].plot(5.5, 5.0, 'ro', markersize=2) # Plot the goal (5.5, 5.0)
axes[0].set_xlabel('X')
axes[0].set_ylabel('Y')
axes[0].set_title('Point Robot in Environment 1')

# Box Robot in Environment 1
axes[1].axis('equal')
axes[1].set_xlim(-6, 6)
axes[1].set_ylim(-6, 6)

positions = [(-4, -4, 4.5, 4), 
             (2, -4, 4, 4), 
             (2, 0.2, 3, 3), 
             (-4, 1, 3.5, 4.5)]
for position in positions:
    plot_rectangle(axes[1], position)

data1 = np.loadtxt('box_env1.txt')
x = data1[:, 0]
y = data1[:, 1]
theta = data1[:, 2]
axes[1].plot(x, y)

axes[1].plot(-5, -5, 'bs', markersize=5) # Plot the start (-5, -5)
axes[1].plot(5.5, 5.0, 'rs', markersize=5) # Plot the goal (5.5, 5.0)
axes[1].set_xlabel('X')
axes[1].set_ylabel('Y')
axes[1].set_title('Box Robot in Environment 1')

# Point Robot in Environment 2
axes[2].axis('equal')
axes[2].set_xlim(-6, 6)
axes[2].set_ylim(-6, 6)

positions = [(-3.5, -3.5, 4.5, 2), 
             (1, -6, 5, 5), 
             (1, -1, 6, 2), 
             (1, 1, 2, 4), 
             (-3, 2, 4, 3), 
             (-7, -0.5, 5, 1.5)]
for position in positions:
    plot_rectangle(axes[2], position)

data = np.loadtxt('point_env2.txt')
axes[2].plot(data[:, 0], data[:, 1], 'o-', markersize=2)
axes[2].plot(-5, -5, 'bo', markersize=2) # Plot the start (-5, -5)
axes[2].plot(5.5, 5.0, 'ro', markersize=2) # Plot the goal (5.5, 5.0)
axes[2].set_xlabel('X')
axes[2].set_ylabel('Y')
axes[2].set_title('Point Robot in Environment 2')

# Box Robot in Environment 2
axes[3].axis('equal')
axes[3].set_xlim(-6, 6)
axes[3].set_ylim(-6, 6)

positions = [(-3.5, -3.5, 4.5, 2), 
             (1, -6, 5, 5), 
             (1, -1, 6, 2), 
             (1, 1, 2, 4), 
             (-3, 2, 4, 3), 
             (-7, -0.5, 5, 1.5)]
for position in positions:
    plot_rectangle(axes[3], position)

data1 = np.loadtxt('box_env2.txt')
x = data1[:, 0]
y = data1[:, 1]
theta = data1[:, 2]
axes[3].plot(x, y)

axes[3].plot(-5, -5, 'bs', markersize=5) # Plot the start (-5, -5)
axes[3].plot(5.5, 5.0, 'rs', markersize=5) # Plot the goal (5.5, 5.0)
axes[3].set_xlabel('X')
axes[3].set_ylabel('Y')
axes[3].set_title('Box Robot in Environment 2')

# Adjust spacing between subplots
plt.tight_layout()

plt.show()
