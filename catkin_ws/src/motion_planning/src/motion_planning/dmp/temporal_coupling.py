#!/usr/bin/env python

import numpy as np


class NoTC:
    def update(self, dmp, dt):
        return 0


class TCVelAccConstrained:

    def __init__(self, gamma_nominal, gamma_a, v_max, a_max, eps=0.001):
        self.gamma_nominal = gamma_nominal
        self.gamma_a = gamma_a
        self.eps = eps
        self.v_max = v_max.reshape((len(v_max), 1))
        self.a_max = a_max.reshape((len(a_max), 1))

    def generate_matrices(self, dmp, dt):
        A = np.vstack((-dmp.z(), dmp.z()))
        B = np.vstack((-self.a_max, -self.a_max))
        C = np.vstack((dmp.h(), -dmp.h()))
        D = np.vstack((-self.v_max, -self.v_max))
        x_next = dmp.x + dmp.f(dmp.x) / dmp.tau * dt
        A_next = np.vstack((-dmp.z(x_next), dmp.z(x_next)))
        C_next = np.vstack((dmp.h(x_next), -dmp.h(x_next)))
        return A, B, C, D, A_next, C_next

    def update(self, dmp, dt):

        A, B, C, D, A_next, C_next = self.generate_matrices(dmp, dt)

        # Acceleration bounds
        i = np.squeeze(A < 0)
        if i.any():
            taud_min_a = np.max(- (B[i] * dmp.tau ** 2 + C[i]) / A[i])
        else:
            taud_min_a = -np.inf
        i = np.squeeze(A > 0)
        if i.any():
            taud_max_a = np.min(- (B[i] * dmp.tau ** 2 + C[i]) / A[i])
        else:
            taud_max_a = np.inf
        # Velocity bounds
        i = range(len(A_next))
        tau_min_v = np.max(-A_next[i] / D[i])
        taud_min_v = (tau_min_v - dmp.tau) / dt
        # Feasibility bounds
        ii = np.arange(len(A_next))[np.squeeze(A_next < 0)]
        jj = np.arange(len(A_next))[np.squeeze(A_next > 0)]
        tau_min_f = -np.inf
        for i in ii:
            for j in jj:
                num = C_next[i] * abs(A_next[j]) + C_next[j] * abs(A_next[i])
                if num > 0:
                    den = abs(B[i] * A_next[j]) + abs(B[j] * A_next[i])
                    tmp = np.sqrt(num / den)
                    if tmp > tau_min_f:
                        tau_min_f = tmp
        taud_min_f = (tau_min_f - dmp.tau) / dt
        # Nominal bound
        taud_min_nominal = (dmp.tau0 - dmp.tau) / dt

        taud_min = np.max((taud_min_a, taud_min_v, taud_min_f, taud_min_nominal))

        # Base update law
        ydd_bar = dmp.h() / (dmp.tau**2 * self.a_max)
        if self.gamma_a > 0:
            pot_a = self.gamma_a * np.sum(ydd_bar ** 2 / np.maximum(1 - ydd_bar ** 2, self.gamma_a * self.eps * np.ones((len(ydd_bar), 1))))
        else:
            pot_a = 0
        #pot_a = self.gamma_a * np.amax(ydd_bar ** 2 / np.maximum(1 - ydd_bar ** 2, self.gamma_a * self.eps * np.ones((len(ydd_bar), 1))))
        taud = self.gamma_nominal * (dmp.tau0 - dmp.tau) + dmp.tau * pot_a

        # Saturate
        taud = np.min((taud, taud_max_a))
        taud = np.max((taud, taud_min))

        return taud
