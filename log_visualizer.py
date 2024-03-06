from ast import pattern
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import re
import numpy as np
from matplotlib.animation import FuncAnimation

# Regex pattern to extract x, y, and z coordinates
# ex. 2024-01-30 20:02:38.607 [DEBUG] (x, y, z, yaw (deg)): (11.488936253187372, 2.5705596286916026, 10.448980907892672, -3.5425534541946733)
# or  2024-01-30 20:02:38.601 [INFO] Global pose from Tag 3: 11.49519167065629, 2.5546613952959873, 10.478145823507184

# Function to parse a line and extract x, y, z coordinates
def parse_line(line):
    if "speed" in line:
        return None
        
    split_line = line.split(": ")
    split_nums = split_line[1].split(", ")

    # remove non-numeric characters
    split_nums = [re.sub('[^0-9.,-]', '', i) for i in split_nums]

    split_nums = [float(i.strip()) for i in split_nums]
    
    x = float(split_nums[0])
    y = float(split_nums[1])
    z = float(split_nums[2])
    return x, y, z

# Function to update the plot in the animation
def update(frame):
    ax.cla()
    ax.set_xlim([-15, 15])
    ax.set_ylim([-15, 15])
    ax.set_zlim([-15, 15])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('3D Animation of XYZ Coordinates')

    # print last point
    print(f"{frame}: ({xs[frame]}, {ys[frame]}, {zs[frame]})", end='\r')
    ax.scatter(xs[:frame], ys[:frame], zs[:frame], color='b', marker='o')

# Read log file and extract x, y, z coordinates
xs = []
ys = []
zs = []

with open('logs/oakd_log.log', 'r') as file:
    for line in file:
        if '[DEBUG]' in line or '[INFO]' in line:  # Filter out lines that do not contain coordinates
            coordinates = parse_line(line)
            if coordinates:
                x, y, z = coordinates
                xs.append(x)
                ys.append(y)
                zs.append(z)

print("Number of coordinates: ", len(xs))

# Set up the 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Create the animation
ani = FuncAnimation(fig, update, frames=len(xs), interval=1)
ani.save('logs/log_visu.gif', writer='imagemagick', fps=30)
# plt.show()
