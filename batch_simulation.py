import matplotlib
from matplotlib import pyplot as plt
from Robot.Omnidirectional import *
import yaml
import numpy as np

with open("config/config_omni.yaml", 'rb') as f:
    conf = yaml.safe_load(f.read())  # load the config file

Ns = [10, 15, 20, 50,100]
x = [[] for _ in range(len(Ns))]
y = [[] for _ in range(len(Ns))]
refx = [[] for _ in range(len(Ns))]
refy = [[] for _ in range(len(Ns))]
plt.figure(figsize=(17,10))
for j, n in enumerate(Ns):
    conf["N"] = n
    rob = Omnidirectional(conf)
    rob.N = n
    rob.MPC.N = n
    print(j)
    error = 0
    for i in range(int(30 * 1 / rob.dt_control)):
        rob.simulate()
        x[j].append(rob.x)
        y[j].append(rob.y)
        refx[j].append(rob.ref[0])
        refy[j].append(rob.ref[1])

for j, n in enumerate(Ns):
    ref_array = np.array([refx[j], refy[j]])
    x_array = np.array([x[j], y[j]])

    # Transpose arrays to make the shape compatible for subtraction if needed
    ref_array = ref_array.T
    x_array = x_array.T

    # Compute the L2 norm difference for corresponding elements
    l2_norms = np.linalg.norm(x_array - ref_array, axis=1, keepdims=True)
    print(f"Combined MSE between refx, x and refy, y: {np.mean(l2_norms)*100, n}")

    plt.plot(x[j], y[j], label=f"Trajectory N = {n}",linewidth=2.5)
plt.plot(refx[0], refy[1], label="Reference Trajectory",color="black",linewidth=2.5)

plt.legend(fontsize=15)

# Adjust layout to ensure labels do not overlap the graph

plt.xlabel('X',fontsize=30)
plt.ylabel('Y',fontsize=30)
plt.xticks(fontsize=25)
plt.yticks(fontsize=25)
plt.grid()
plt.title('Software in The Loop Trajectories With Augmented Model, Velocity = 0.2 m/s', fontsize=28)
plt.legend(loc='upper right', fontsize=18)
plt.xlim(-0.85, 1.2)
plt.show()
