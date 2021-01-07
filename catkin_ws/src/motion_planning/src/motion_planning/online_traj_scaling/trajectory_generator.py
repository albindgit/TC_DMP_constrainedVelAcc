#!/usr/bin/env python

import matplotlib.pyplot as plt
import yaml
from gmr_oa import GMRwOA
import numpy as np

class TrajectoryGenerator:
    def __init__(self, dyn_sys):
        # parameters
        self.dyn_sys = dyn_sys
        # state
        self.x = None
        self.s = None
        self.ds = None
        self.sat_flag = None
        # Output state
        self.pos = None
        self.vel = None
        self.acc = None
        # desired path
        self.path = None

        self.reset()

    def reset(self):
        self.x = self.dyn_sys.x0
        self.s = 0
        self.ds = 1
        self.sat_flag = 0
        self.pos = self.dyn_sys.g(self.x)
        self.vel = self.dyn_sys.gx(self.x).dot(self.dyn_sys.f(self.x))
        self.acc = self.dyn_sys.dx_gxf(self.x).dot(self.dyn_sys.f(self.x))

    def step(self, dt):
        # Derivative
        dx = self.dyn_sys.f(self.x) * self.ds
        dds, self.sat_flag = self.traj_scaling()

        # Integrate
        self.x += dx * dt
        self.s += self.ds * dt
        self.ds += dds * dt

        # Update reference
        self.pos = self.dyn_sys.g(self.x)
        self.vel = self.dyn_sys.gx(self.x).dot(self.dyn_sys.f(self.x))
        self.acc = self.dyn_sys.gx(self.x).dot(self.dyn_sys.f(self.x)) * dds + self.dyn_sys.dx_gxf(
            self.x).dot(self.dyn_sys.f(self.x)) * self.ds ** 2

    def roll_out(self, dt, T = None):
        self.reset()
        pos = np.array([self.pos])
        vel = np.array([self.vel])
        acc = np.array([self.acc])
        t = np.array([0])
        s = np.array([0])
        # for k in range(1, round(T/dt)+1):
        k = 0
        while True:
            k += 1
            self.step(dt)
            t = np.append(t, k*dt)
            s = np.append(s, self.s)
            pos = np.append(pos, np.array([self.pos]), axis=0)
            vel = np.append(vel, np.array([self.vel]), axis=0)
            acc = np.append(acc, np.array([self.acc]), axis=0)
            if T and k*dt > T:
                break
            if not T and self.s > self.dyn_sys.s_end:
                break
        return t, s, pos, vel, acc

    def traj_scaling(self):
        dds = 0
        sat_flag = 0

        return dds, sat_flag

if __name__ == "__main__":
    with open("example.yaml", 'r') as stream:
        try:
            params = yaml.safe_load(stream)
            params['sigma'] = [[[el * 1e3 for el in row] for row in col] for col in params['sigma']]
            gmr = GMRwOA(params)
            tg = TrajectoryGenerator(gmr)
            t, s, pos, vel, acc = tg.roll_out(0.01)
            plt.plot(0.0032*pos[:, 0], 0.0026*pos[:, 1] + 0.7)
            # plt.plot(s, pos[:, 0])
            # plt.plot(s, pos[:, 1])
            plt.show()
        except yaml.YAMLError as exc:
            print(exc)
