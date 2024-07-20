import copy

from Robot.MPC import *


from Robot.Motor import *


class Omnidirectional:
    def __init__(self, cfg):
        self.cfg = cfg
        self.theta = 0
        self.v = 0
        self.vn = 0
        self.vx = 0
        self.vy = 0
        self.vx_ref = 0
        self.vy_ref = 0
        self.v_ref = 0
        self.vn_ref = 0
        self.vref = 0
        self.w = 0
        self.wref = 0

        self.dt_sim = cfg["dt_sim"]
        self.dt_control = cfg["dt_control"]
        self.r = cfg["wheel_radius"]
        self.length = cfg["robot_length"]
        self.width = cfg["robot_width"]
        self.t = 0
        self.i = 0
        self.n_motors = 4
        self.motors = [Motor(self.cfg) for _ in range(4)]
        self.kinematics_matrix = 1 / 4 * np.array([
            [1, -1, 1, -1],
            [-1, -1, 1, 1],
            [-1 / (self.length + self.width), -1 / (self.length + self.width), -1 / (self.length + self.width),
             -1 / (self.length + self.width)]
        ])
        self.inverse_kinematics = np.linalg.pinv(self.kinematics_matrix)
        self.N = cfg['N']
        self.x_trajectory = np.load(cfg['x_trajectory'])
        self.y_trajectory = np.load(cfg['y_trajectory'])
        self.x = self.x_trajectory[0]
        self.y = self.y_trajectory[0]
        self.augmented = cfg['augmented']
        self.MPC = MPC(cfg, self.augmented)
        self.k = 0
        self.ref = np.array([0, 0])

    def simulate(self):
        self.step()

    def step(self, action=None):
        self.k += 1
        k = self.k % len(self.x_trajectory)

        # Create rx and ry arrays with cyclic behavior
        rx = [self.x_trajectory[(k + i) % len(self.x_trajectory)] for i in range(self.N)]
        ry = [self.y_trajectory[(k + i) % len(self.y_trajectory)] for i in range(self.N)]
        if self.augmented:
            x_bar = [self.x, self.y, self.vx, self.vy, self.vx_ref, self.vy_ref]
        else:
            x_bar = [self.x, self.y, self.vx_ref, self.vy_ref]

        r = np.zeros((2 * self.N, 1))
        for k in range(self.N):
            r[2 * k] = rx[k]
            r[2 * k + 1] = ry[k]
        self.ref = np.array([copy.deepcopy(rx[0]), copy.deepcopy(ry[0])])
        delta_u = self.MPC.compute(np.array(x_bar), r)
        self.vx_ref += delta_u[0, 0]
        self.vy_ref += delta_u[1, 0]
        self.update()

        for i in range(int(self.dt_control / self.dt_sim)):
            for j in range(self.n_motors):
                self.motors[j].update()

            self.kinematics()
            self.t += self.dt_sim
            self.i += 1

    def update(self):
        self.transformToRobotRef()
        vref = np.array([self.v_ref, self.vn_ref, self.wref])
        wheel_vel = self.inverse_kinematics @ np.transpose(vref)
        for i in range(self.n_motors):
            self.motors[i].control(wheel_vel[i] / self.r)

    def kinematics(self):
        wheel_v = np.array([self.motors[i].w * self.r for i in range(self.n_motors)])
        vel = self.kinematics_matrix @ wheel_v
        self.v = vel[0]
        self.vn = vel[1]
        self.w = vel[2]
        self.transformToGlobalRef()

        self.x = self.x + self.vx * self.dt_sim
        self.y = self.y + self.vy * self.dt_sim
        self.theta = self.theta + self.w * self.dt_sim

    def transformToRobotRef(self):
        self.v_ref = self.vx_ref * np.cos(self.theta) + self.vy_ref * np.sin(self.theta)
        self.vn_ref = -self.vx_ref * np.sin(self.theta) + self.vy_ref * np.cos(self.theta)

    def transformToGlobalRef(self):
        self.vx = self.v * np.cos(self.theta) - self.vn * np.sin(self.theta)
        self.vy = self.v * np.sin(self.theta) + self.vn * np.cos(self.theta)


