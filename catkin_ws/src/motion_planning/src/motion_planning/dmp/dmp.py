#!/usr/bin/env python

import numpy as np


class DMP:
    def __init__(self, k=100, d=20, a_s=1, n_bfs=100):
        # parameters
        self.k = k
        self.d = d
        self.a_s = a_s
        self.n_bfs = n_bfs

        # for obstacle avoidance
        self.g_obs = 1000.
        self.b_obs = 20. / np.pi
        self.a_obs = 0.2

        # trajectory parameters
        self.n = 0
        self.y0 = None
        self.tau0 = None
        self.g = None
        self.tau = None

        # initialize basis functions for LWR
        self.w = None
        self.centers = None
        self.widths = None

        # state
        self.x = None
        self.theta = None
        self.pos = None
        self.vel = None
        self.acc = None

        # desired path
        self.path = None

    def reset(self):
        self.x = np.vstack((np.zeros((self.n, 1)), self.y0, 1.))
        self.tau = self.tau0
        self.theta = 0
        self.pos = self.y0
        self.vel = 0 * self.y0
        self.acc = 0 * self.y0

    def z(self, x=None):
        if x is None:
            x = self.x
        return x[0:self.n]

    def y(self, x=None):
        if x is None:
            x = self.x
        return x[self.n:2 * self.n]

    def s(self, x=None):
        if x is None:
            x = self.x
        return x[2 * self.n]

    def set_goal(self, g):
        self.g = g

    def set_tau(self, tau):
        self.tau = tau

    def psi(self, s, i=None):
        if i is not None and len(s) == 1:
            return np.exp(-1 / (2 * self.widths[i] ** 2) * (s - self.centers[i]) ** 2)
        if i is None:
            return np.exp(-1 / (2 * self.widths ** 2) * (s - self.centers) ** 2)

    def h(self, x=None):
        if x is None:
            x = self.x
        psi = self.psi(self.s(x)).reshape((self.n_bfs, 1))
        v = (self.w.dot(psi)) / np.maximum(np.sum(psi), 1e-8) * (self.g - self.y0) * self.s(x)
        h = self.k * (self.g - self.y(x)) - self.d * self.z(x) + v
        return h

    def f(self, x=None):
        if x is None:
            x = self.x
        return np.vstack((self.h(x), self.z(x), -self.a_s * self.s(x)))

    def obstacle_avoidance(self):
        p = np.zeros(self.pos.shape)

        for obstacle in self.obstacles:
            # calculate vector to object relative to body
            o = np.squeeze(obstacle)
            x = np.squeeze(self.pos)
            v = np.squeeze(self.vel)
            # if we're moving
            if np.linalg.norm(self.vel) > 1e-8 and np.linalg.norm(o - x) < self.a_obs:
                # print("Dist: " + str(np.linalg.norm(o - x)) + "at position: " + str(x))
                phi = np.arccos( np.dot(o - x, v) / (np.linalg.norm(o - x) * np.linalg.norm(v)) )
                dphi = self.g_obs * phi * np.exp(-self.b_obs * abs(phi))

                r = np.cross(o - x, v)
                r = r / np.linalg.norm(r)
                R = np.array([[0,-r[2],r[1]], [r[2],0,r[0]], [-r[1],r[0],0]]) + np.outer(r,r)
                pval = np.nan_to_num(np.dot(R, v) * dphi)

                # check to see if the distance to the obstacle is further than
                # the distance to the target, if it is, ignore the obstacle
                if np.linalg.norm(o - x) > np.linalg.norm(self.g - self.pos):
                    pval = p * 0.

                p += pval.reshape(self.pos.shape)
        return p

    def step(self, dt):
        # Update state
        self.x = self.x + self.f() / self.tau * dt
        self.theta = self.theta + 1 / self.tau * dt

        # Extract trajectory state
        vel_prev = self.vel
        self.pos = self.y()
        self.vel = self.z() / self.tau
        self.acc = (self.vel - vel_prev) / dt

    def fit(self, y_demo, t_demo):
        # Set target trajectory parameters
        t_demo = t_demo.reshape(1, len(t_demo))
        self.n = y_demo.shape[0]
        self.path = y_demo
        self.y0 = y_demo[:, 0].reshape((self.n, 1))
        self.g = y_demo[:, -1].reshape((self.n, 1))
        self.tau0 = t_demo[0, -1]

        # Set basis functions
        t_centers = np.linspace(0, self.tau0, self.n_bfs, endpoint=True)
        self.centers = np.exp(-self.a_s / self.tau0 * t_centers)
        widths = np.abs((np.diff(self.centers)))
        self.widths = np.concatenate((widths, [widths[-1]]))

        # Calculate derivatives
        yd_demo = (y_demo[:, 1:] - y_demo[:, :-1]) / (t_demo[0, 1:] - t_demo[0, :-1])
        yd_demo = np.concatenate((yd_demo, np.zeros((self.n, 1))), axis=1)
        ydd_demo = (yd_demo[:, 1:] - yd_demo[:, :-1]) / (t_demo[0, 1:] - t_demo[0, :-1])
        ydd_demo = np.concatenate((ydd_demo, np.zeros((self.n, 1))), axis=1)

        # Compute weights
        s_seq = np.exp(-self.a_s / self.tau0 * t_demo)
        self.w = np.zeros((self.n, self.n_bfs))
        for i in range(self.n):
            if abs(self.g[i] - self.y0[i]) < 1e-5:
                continue
            f_gain = s_seq * (self.g[i] - self.y0[i])
            f_target = self.tau0 ** 2 * ydd_demo[i, :] - self.k * (
                        self.g[i] - y_demo[i, :]) + self.d * self.tau0 * yd_demo[i, :]
            for j in range(self.n_bfs):
                psi_j = self.psi(s_seq, j)
                num = f_gain.dot((psi_j * f_target).T)
                den = f_gain.dot((psi_j * f_gain).T)
                if abs(den) < 1e-6:
                    continue
                self.w[i, j] = num / den

        # Reset state
        self.reset()
