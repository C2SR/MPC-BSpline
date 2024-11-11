import bagpy
from bagpy import bagreader
import pandas as pd
from matplotlib import pyplot as plt

from Robot.Omnidirectional import *
import yaml
import numpy as np




bags = ['bags/aug02.bag', 'bags/aug04.bag', 'bags/aug06.bag','bags/aug065.bag']
vel = ["0.2m/s","0.4m/s","0.6m/s","0.65m/s"]
news = [True,True,False,True]
x = [[] for _ in range(len(bags))]
y = [[] for _ in range(len(bags))]
refx = [[] for _ in range(len(bags))]
refy = [[] for _ in range(len(bags))]
offsets = [625+20,318+10,215,1000,1000]
#offsets = [625+20,318+10,214+10,200+10,300+10]
# Read the topic
plt.figure(figsize=(17,10))
for j,bag_file in enumerate(bags):

    b = bagreader(bag_file)
    if not news[j] :
        x_data = b.message_by_topic('/unnamed_robot/x')
        y_data = b.message_by_topic('/unnamed_robot/y')
        rx0_data = b.message_by_topic('/unnamed_robot/rx0')
        ry0_data = b.message_by_topic('/unnamed_robot/ry0')

        # Convert to pandas DataFrame
        x_df = pd.read_csv(x_data)['data']
        y_df = pd.read_csv(y_data)['data']
        rx0_df = pd.read_csv(rx0_data)['data']
        ry0_df = pd.read_csv(ry0_data)['data']
    else:
        data = b.message_by_topic('/unnamed_robot/my_data')
        data_df = pd.read_csv(data)
        x_df = data_df['x']
        y_df = data_df['y']
        rx0_df = data_df['z']
        ry0_df = data_df['w']

    error_x = 0
    error_y = 0
    for i in range(len(x_df)-10):
        x[j].append(x_df[i])
        y[j].append(y_df[i])
        refx[j].append(rx0_df[i])
        refy[j].append(ry0_df[i])





for j, n in enumerate(vel):
    ref_array = np.array([refx[j], refy[j]])
    x_array = np.array([x[j], y[j]])
    error = 0
    for i in range(2,len(x[j])-1):
        error += np.sqrt((x[j][i] - refx[j][i]) ** 2 + (y[j][i] - refy[j][i]) ** 2)
    print(error / len(x[j]) * 100)
    # Transpose arrays to make the shape compatible for subtraction if needed
    ref_array = ref_array.T
    x_array = x_array.T

    # Compute the L2 norm difference for corresponding elements
    l2_norms = np.linalg.norm(x_array - ref_array, axis=1, keepdims=True)
    print(f"Combined MSE between refx, x and refy, y: {np.mean(l2_norms) * 100, n}")

    plt.plot(x[j][0:offsets[j]], y[j][0:offsets[j]],label=f" Velocity = {n}",linewidth=2.5)
plt.plot(refx[0], refy[0],label="Reference Trajectory",color="black",linewidth=3)


plt.legend(fontsize=17)

# Adjust layout to ensure labels do not overlap the graph

plt.xlabel('X',fontsize=30)
plt.ylabel('Y',fontsize=30)
plt.xticks(fontsize=25)
plt.yticks(fontsize=25)
plt.grid()
plt.title('Hardware in the Loop Trajectories, Augmented Model',fontsize=30)
plt.legend(loc='upper right',fontsize=18)
plt.xlim(-0.85, 1.2)

plt.show()