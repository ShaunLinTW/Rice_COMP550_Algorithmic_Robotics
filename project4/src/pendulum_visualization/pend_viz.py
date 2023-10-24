'''
///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: Jason Zhang(jz118), Shaun Lin(hl116)
//////////////////////////////////////
'''

import matplotlib.pyplot as plt
import numpy as np

x = []
y = []

# Read the data from the file
with open('RRT.txt', 'r') as file:
    for line in file:
        parts = line.strip().split(' ')
        if len(parts) == 2:
            x.append(float(parts[0]))
            y.append(float(parts[1]))

# Create a curve plot
plt.plot(x, y, marker='o', linestyle='-')

# Add labels and title
plt.xlabel('theta')
plt.ylabel('omega')
plt.title('Pendulum Planning using RRT')
plt.savefig('RRT.png')
plt.close()

x = []
y = []

with open('KPIECE.txt', 'r') as file:
    for line in file:
        parts = line.strip().split(' ')
        if len(parts) == 2:
            x.append(float(parts[0]))
            y.append(float(parts[1]))

# Create a curve plot
plt.plot(x, y, marker='o', linestyle='-')

# Add labels and title
plt.xlabel('theta')
plt.ylabel('omega')
plt.title('Pendulum Planning using KPIECE')
plt.savefig('KPIECE.png')