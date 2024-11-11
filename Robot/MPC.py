
import numpy as np


class MPC:
    def __init__(self, cfg, augmented=False):
        self.cfg = cfg
        if augmented:
            self.A = np.block([[np.eye(2), np.eye(2) * 0.01813],
                               [np.eye(2) * 0, np.eye(2) * 0.8187]])
            n = self.A.shape[0]

            self.B = np.block([[np.eye(2) * 0.001873],
                               [np.eye(2) * 0.1813]])
            m = self.B.shape[1]
            self.C = np.block([np.eye(2), 0 * np.eye(2)])
        else:
            self.A = np.eye(2)
            n = self.A.shape[0]

            self.B = np.eye(2) * 1 / 50
            m = self.B.shape[1]
            self.C = np.eye(2)

        self.x_bar = np.array([0, 0, 0])
        self.A_bar = np.block([[self.A, self.B],
                               [np.zeros((2, n)), np.eye(m)]])
        self.B_bar = np.block([[self.B],
                               [np.eye(2)]])
        self.C_bar = np.block([self.C, 0 * np.eye(2)])
        self.Q = np.array(cfg["Q"])
        self.R = np.array(cfg["R"])
        self.N = cfg["N"]
        I = np.eye(self.N)

        self.Q_barbar = np.kron(I, np.transpose(self.C_bar) @ self.Q @ self.C_bar)
        self.T_barbar = np.kron(I, self.Q @ self.C_bar)
        #S = control.dare(self.A, self.B, self.Q, self.R)[0]
        #self.Q_barbar[-self.A_bar.shape[0] - 1:-1, -self.A_bar.shape[0] - 1:-1] = np.transpose(self.C_bar) @ S @ self.C_bar
        #S_C = S @ self.C_bar
        #self.T_barbar[-S_C.shape[0] - 1:-1, -S_C.shape[1] - 1:-1] = S_C
        self.R_barbar = np.kron(I, self.R)
        self.C_barbar = np.kron(I, self.B_bar)

        n = self.A_bar.shape[0]
        m = self.B_bar.shape[1]
        for i in range(self.N):
            for j in range(self.N):
                if i - j >= 0:
                    self.C_barbar[i * n: (i + 1) * n, j * m: (j + 1) * m] = np.linalg.matrix_power(self.A_bar,
                                                                                                   (i - j)) @ self.B_bar

        self.A_hathat = np.zeros((self.N * n, n))
        for i in range(self.N):
            self.A_hathat[i * n:(i + 1) * n, 0:n] = np.linalg.matrix_power(self.A_bar, i + 1)

        self.H_barbar = np.transpose(self.C_barbar) @ self.Q_barbar @ self.C_barbar + self.R_barbar
        self.F_barbar = np.block(
            [[np.transpose(self.A_hathat) @ self.Q_barbar @ self.C_barbar], [-self.T_barbar @ self.C_barbar]])

    def compute(self, x_bar, r):
        self.x_bar = np.transpose(np.atleast_2d(x_bar))

        return - np.linalg.inv(self.H_barbar) @ np.transpose(self.F_barbar) @ np.block([[self.x_bar], [r]])

    def predict(self, delta_u, x0):
        predict = self.C_barbar @ delta_u + self.A_hathat @ np.transpose(np.atleast_2d(x0))
        xp = [predict[6 * i + 1, 0] for i in range(self.N)]
        state_array = []
        state = np.transpose(np.atleast_2d(x0))

        for i in range(self.N):
            state = (self.A_bar @ state) + self.B_bar @ delta_u[2 * i: 2 * (i + 1), :]
            state_array.append(state[1, 0])

        e = np.array(xp) - np.array(state_array)
        return predict
