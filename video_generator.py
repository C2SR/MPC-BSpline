import bagpy
from bagpy import bagreader
import pandas as pd
from matplotlib import pyplot as plt
import matplotlib.animation as animation
from Robot.Omnidirectional import *
import yaml
import numpy as np

bags = ['bags/aug065.bag', 'bags/int065.bag']
vel = ["0.2m/s"]
news = [True, True]
x = [[] for _ in range(len(bags))]
y = [[] for _ in range(len(bags))]
refx = [[] for _ in range(len(bags))]
refy = [[] for _ in range(len(bags))]

# Read the topic

for j, bag_file in enumerate(bags):

    b = bagreader(bag_file)
    if not news[j]:
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
    for i in range(len(x_df) - 10):
        x[j].append(x_df[i])
        y[j].append(y_df[i])
        refx[j].append(rx0_df[i])
        refy[j].append(ry0_df[i])

offsets = []

for i in range(len(x[0])):

    if np.sqrt((-0.705 - refx[0][i]) ** 2 + (0 - refy[0][i]) ** 2) < 0.006:
        offsets.append(i)
        break

for i in range(len(x[1])):
    if refx[1][i] == refx[0][offsets[0]] and refy[1][i] == refy[0][offsets[0]]:
        offsets.append(i)
        break

print(offsets)

x[0] = x[0][offsets[0]:offsets[0]+318+10]
y[0] = y[0][offsets[0]:offsets[0]+318+10]
x[1] = x[1][offsets[1]:offsets[1]+318+10]
y[1] = y[1][offsets[1]:offsets[1]+318+10]
refx[0] = refx[0][offsets[0]:-1]
refy[0] = refy[0][offsets[0]:-1]
# Animation function
image_path = "splines/canvas.png"
img = plt.imread(image_path)
img_extent = [-850/1000, 850/1000,
              -600/1000, 600/1000]
fig, ax = plt.subplots()

line1, = ax.plot([], [], 'r-', label='Augmented Model',linewidth=3)
line2, = ax.plot([], [], 'b-', label='Integrator Model',linewidth=3)
line_ref, = ax.plot([], [], 'k--', label='Reference',linewidth=3)
circle1 = plt.Circle((0, 0), 0.01, color='r')
circle2 = plt.Circle((0, 0), 0.01, color='b')
circle_ref = plt.Circle((0, 0), 0.01, color='k')
ax.add_patch(circle1)
ax.add_patch(circle2)
ax.add_patch(circle_ref)
ax.legend(fontsize=10)


def init():
    ax.set_xlim(-0.85, 0.85)
    ax.set_ylim(-0.6, 0.6)
    ax.imshow(img, extent=img_extent, aspect='auto', zorder=-1)
    ax.tick_params(axis='both', which='major', labelsize=15)

    return line1, line2, line_ref, circle1, circle2, circle_ref


def update(frame):
    line1.set_data(x[0][:frame + 1], y[0][:frame + 1])
    line2.set_data(x[1][:frame + 1], y[1][:frame + 1])
    line_ref.set_data(refx[0][:frame + 1], refy[0][:frame + 1])

    circle1.set_center((x[0][frame], y[0][frame]))
    circle2.set_center((x[1][frame], y[1][frame]))
    circle_ref.set_center((refx[0][frame], refy[0][frame]))

    return line1, line2, line_ref, circle1, circle2, circle_ref


ani = animation.FuncAnimation(fig, update, frames=range(min(len(x[0]), len(x[1]), len(refx[0]))),
                              init_func=init, blit=True, interval=40)  # 40ms interval for 1/25s sampling time

Writer = animation.writers['ffmpeg']
writer = Writer(fps=25, metadata=dict(artist='Me'), bitrate=1800)
ani.save("vel065.mp4", writer=writer)

plt.show()
