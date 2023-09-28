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

# Create a figure with 1x2 subplots
fig, axes = plt.subplots(1, 2, figsize=(8, 4))
axes = axes.flatten()

# Environment 1
axes[0].axis('equal')
axes[0].set_xlim(-6, 6)
axes[0].set_ylim(-6, 6)

positions = [(-4, -4, 4.5, 4), 
             (2, -4, 4, 4), 
             (2, 0.2, 3, 3), 
             (-4, 1, 3.5, 4.5)]
for position in positions:
    plot_rectangle(axes[0], position)

axes[0].plot(-5, -5, 'bo', markersize=5) # Plot the start (-5, -5)
axes[0].plot(5.5, 5.0, 'ro', markersize=5) # Plot the goal (5.5, 5.0)
axes[0].set_xlabel('X')
axes[0].set_ylabel('Y')
axes[0].set_title('Environment 1')

# Environment 2
axes[1].axis('equal')
axes[1].set_xlim(-6, 6)
axes[1].set_ylim(-6, 6)

positions = [(-3.5, -3.5, 4.5, 2), 
             (1, -6, 5, 5), 
             (1, -1, 6, 2), 
             (1, 1, 2, 4), 
             (-3, 2, 4, 3), 
             (-7, -0.5, 5, 1.5)]
for position in positions:
    plot_rectangle(axes[1], position)

axes[1].plot(-5, -5, 'bs', markersize=8) # Plot the start (-5, -5)
axes[1].plot(5.5, 5.0, 'rs', markersize=8) # Plot the goal (5.5, 5.0)
axes[1].set_xlabel('X')
axes[1].set_ylabel('Y')
axes[1].set_title('Environment 2')

# Adjust spacing between subplots
plt.tight_layout()

plt.show()
