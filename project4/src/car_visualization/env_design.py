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

PI = 3.14159265358979323846

# Function to plot rectangles
def plot_rectangle(ax, position):
    rect = Rectangle(position[:2], position[2], position[3], fill=False)
    ax.add_patch(rect)

# Create a figure with 1x2 subplots
fig, axes = plt.subplots(1, 2, figsize=(8, 4))
axes = axes.flatten()

# Car Robot Environment
axes[0].axis('equal')
axes[0].set_xlim(-10, 10)
axes[0].set_ylim(-10, 10)

positions = [(-11, -6, 8, 10), 
             (3, -6, 8, 10), 
             (-7, 8, 14, 3),
             (-1, -6, 2, 10)]

for position in positions:
    plot_rectangle(axes[0], position)

axes[0].plot(-8, -8, 'bs', markersize=7, label='start') # Plot the start (-8, -8)
axes[0].plot(9, 9, 'rs', markersize=7, label='goal') # Plot the goal (9, 9)
axes[0].legend(loc='best')
axes[0].set_xlabel('South Street')
axes[0].set_ylabel('West Street')
axes[0].set_title('Car Environment')

# Pendulum Environment
axes[1].axis('equal')
axes[1].set_xlim(-5, 5)
axes[1].set_ylim(-5, 5)

axes[1].plot(-PI/2, 0, 'bo', markersize=5, label='start') # Plot the start (-PI/2, 0)
axes[1].plot(PI/2, 0, 'ro', markersize=5, label='goal') # Plot the goal (PI/2, 0)
axes[1].legend(loc='upper right')
axes[1].set_xlabel('theta')
axes[1].set_ylabel('omega')
axes[1].set_title('Pendulum Environment')

# Adjust spacing between subplots
plt.tight_layout()

plt.show()
