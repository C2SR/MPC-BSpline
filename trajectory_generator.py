import numpy as np
from scipy import interpolate
import matplotlib.pyplot as plt
import networkx as nx
import yaml
import argparse

parser = argparse.ArgumentParser(description='MPC')
parser.add_argument('--map', type=str, default="splines/map.yaml", help='Map YAML')
parser.add_argument('--k', type=int, default=6, help='B-Spline Order')
parser.add_argument('--v', type=float, default=6, help='Robot Nominal Speed')
parser.add_argument('--path', type=list, default=[21, 17, 10, 34, 0, 1, 2, 3, 4, 6, 13, 20, 22, 23, 24, 25, 26, 35, 16, 9, 5, 31, 30, 4, 6, 13, 20, 22,
                 33, 32, 21], help='B-Spline Control Points')
parser.add_argument('--f', type=float, default=50, help='Control Cycle Frequency')
parser.add_argument('--file', type=str, default='trajectory', help='Output File')
parser.add_argument('--folder', type=str, default='config/', help='Output Folder')





args = parser.parse_args()
with open(args.map, "r") as yaml_file:
    data = yaml.safe_load(yaml_file)
G = nx.DiGraph()
# Add nodes to the graph
for node_data in data['nodes']:
    node_id = node_data['id']
    node_x = node_data['x']
    node_y = node_data['y']
    G.add_node(node_id, x=node_x, y=node_y)
# Add edges to the graph
for edge_data in data['edges']:
    source = edge_data['source']
    target = edge_data['target']
    source_x, source_y = G.nodes[source]["x"] / 1000, G.nodes[source]["y"] / 1000
    target_x, target_y = G.nodes[target]["x"] / 1000, G.nodes[target]["y"] / 1000
    distance = np.sqrt((target_x - source_x) ** 2 + (target_y - source_y) ** 2)
    G.add_edge(source, target, weight=distance)
    G.add_edge(target, source, weight=distance)

shortest_path = args.path
x = [G.nodes[node]["x"] for node in shortest_path]
y = [G.nodes[node]["y"] for node in shortest_path]
n = len(x)
k = args.k
t = np.zeros(n + k )
step = 1 / (n - k +1)

for i in range(len(t)):
    if i + 1 < k:
        t[i] = 0
    if i + 1 >= k and i + 1 < (n + 2):
        t[i + 1] = t[i] + step
    if i + 1 >= n + 2:
        t[i] = 1

tck_x = [t, x, k-1]
tck_y = [t, y, k-1]
xx = np.linspace(0, 1, 10000, endpoint=True)  # 50000
yy = np.linspace(0, 1, 10000, endpoint=True)
x_path = interpolate.splev(xx, tck_x)
x_path_der = interpolate.splev(xx, tck_x, der=1)
y_path = interpolate.splev(yy, tck_y)
y_path_der = interpolate.splev(yy, tck_y, der=1)
speed = args.v
dt = 1 / args.f
dist = speed * dt
x_trajectory = [x_path[0]]
y_trajectory = [y_path[0]]

for i in range(len(x_path) - 1):
    start = np.array([x_trajectory[-1], y_trajectory[-1]]) / 1000
    end = np.array([x_path[i], y_path[i]]) / 1000
    dist_between_points = np.linalg.norm(start - end, 2)
    if dist_between_points > dist - dist * 0.01:
        x_trajectory.append(x_path[i])
        y_trajectory.append(y_path[i])

plt.plot(x_path, y_path, 'b-', lw=4, alpha=0.4, label='BSpline')
x_trajectory = np.array(x_trajectory) / 1000
y_trajectory = np.array(y_trajectory) / 1000
np.save(args.folder+'x_'+args.file+'.npy', x_trajectory)
np.save(args.folder+'y_'+args.file+'.npy', y_trajectory)
data = {
    'x_'+args.file: x_trajectory.tolist(),
    'y_'+args.file: y_trajectory.tolist()
}
with open(args.folder+args.file+'.yaml', 'w') as file:
    yaml.dump(data, file)
plt.plot(x, y,  label='Control polygon', marker='o', markerfacecolor='red')
plt.show()
