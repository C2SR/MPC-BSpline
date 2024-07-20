import matplotlib
from matplotlib import pyplot as plt
from Robot.Omnidirectional import *
import yaml
import numpy as np
from tqdm import trange
with open("config/config_omni.yaml", 'rb') as f:
    conf = yaml.safe_load(f.read())  # load the config file

x = []
y = []
refx = []
refy = []
plt.figure(figsize=(17,10))

sim_len = conf['sim_len']
rob = Omnidirectional(conf)
for i in trange(int(sim_len * 1 / rob.dt_control)):
    rob.simulate()
    x.append(rob.x)
    y.append(rob.y)
    refx.append(rob.ref[0])
    refy.append(rob.ref[1])

ref_array = np.array([refx, refy])
x_array = np.array([x, y])

ref_array = ref_array.T
x_array = x_array.T

l2_norms = np.linalg.norm(x_array - ref_array, axis=1, keepdims=True)
print(f"Combined MSE between refx, x and refy, y: {np.mean(l2_norms)*100}")

plt.plot(x, y,label='Robot Trajectory',linewidth=2.5)
plt.plot(refx, refy, label="Reference Trajectory",color="black",linewidth=2.5)

plt.legend(fontsize=15)

plt.xlabel('X',fontsize=30)
plt.ylabel('Y',fontsize=30)
plt.xticks(fontsize=25)
plt.yticks(fontsize=25)
plt.grid()
plt.title('Software in The Loop Trajectory', fontsize=28)
plt.legend(loc='upper right', fontsize=18)
plt.xlim(-0.85, 1.2)
plt.show()
