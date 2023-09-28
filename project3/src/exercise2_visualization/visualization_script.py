from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Polygon

# Function to plot rectangles
def plot_rectangle(ax, position):
    rect = Rectangle(position[:2], position[2], position[3], fill=False)
    ax.add_patch(rect)

# Function to plot rotated rectangles
# def plot_rotated_rectangle(ax, x, y, theta):
#     pgon = Polygon(
#         np.array([[x + 0.125, x - 0.125, x - 0.125, x + 0.125],
#                   [y - 0.125, y - 0.125, y + 0.125, y + 0.125]]).T,
#         closed=True,
#         edgecolor='k'
#     )
#     pgon.set_transform(plt.gca().transData)
#     pgon.set_angle(theta * 180 / np.pi)
#     ax.add_patch(pgon)

# Create a figure with 2x2 subplots
fig, axes = plt.subplots(2, 2, figsize=(5, 5))
axes = axes.flatten()

# Point Robot in Environment 1
axes[0].axis('equal')
axes[0].set_xlim(-5, 5)
axes[0].set_ylim(-5, 5)

positions = [(-5, -3, 5, 2), 
             (3, 0.5, 2, 1), 
             (1.5, 1, 1, 2), 
             (-2, 0, 2, 3)]
for position in positions:
    plot_rectangle(axes[0], position)

data = np.loadtxt('point_env1.txt')
axes[0].plot(data[:, 0], data[:, 1], 'o-')
axes[0].set_xlabel('X')
axes[0].set_ylabel('Y')
axes[0].set_title('Point Robot in Environment 1')

# Box Robot in Environment 1
axes[1].axis('equal')
axes[1].set_xlim(-5, 5)
axes[1].set_ylim(-5, 5)

positions = [(-5, -3, 5, 2), 
             (3, 0.5, 2, 1), 
             (1.5, 1, 1, 2), 
             (-2, 0, 2, 3)]
for position in positions:
    plot_rectangle(axes[1], position)

data1 = np.loadtxt('box_env1.txt')
x = data1[:, 0]
y = data1[:, 1]
theta = data1[:, 2]
axes[1].plot(x, y)

# for i in range(len(x)):
#     plot_rotated_rectangle(axes[1], x[i], y[i], theta[i])

axes[1].set_xlabel('X')
axes[1].set_ylabel('Y')
axes[1].set_title('Box Robot in Environment 1')

# Point Robot in Environment 2
axes[2].axis('equal')
axes[2].set_xlim(-5, 5)
axes[2].set_ylim(-5, 5)

positions = [(1, -4, 1, 4), 
             (-2, -5, 1, 3), 
             (1, 1, 1, 3), 
             (1, 0, 5, 1), 
             (-3, 2, 4, 1), 
             (-5, -0.5, 4.5, 1)]
for position in positions:
    plot_rectangle(axes[2], position)

data = np.loadtxt('point_env2.txt')
axes[2].plot(data[:, 0], data[:, 1], 'o-')
axes[2].set_xlabel('X')
axes[2].set_ylabel('Y')
axes[2].set_title('Point Robot in Environment 2')

# Box Robot in Environment 2
axes[3].axis('equal')
axes[3].set_xlim(-5, 5)
axes[3].set_ylim(-5, 5)

positions = [(1, -4, 1, 4), 
             (-2, -5, 1, 3), 
             (1, 1, 1, 3), 
             (1, 0, 5, 1), 
             (-3, 2, 4, 1), 
             (-5, -0.5, 4.5, 1)]
for position in positions:
    plot_rectangle(axes[3], position)

data1 = np.loadtxt('box_env2.txt')
x = data1[:, 0]
y = data1[:, 1]
theta = data1[:, 2]
axes[3].plot(x, y)

# for i in range(len(x)):
#     plot_rotated_rectangle(axes[3], x[i], y[i], theta[i])

axes[3].set_xlabel('X')
axes[3].set_ylabel('Y')
axes[3].set_title('Box Robot in Environment 2')

# Adjust spacing between subplots
plt.tight_layout()

plt.show()
