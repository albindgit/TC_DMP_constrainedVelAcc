#!/usr/bin/env python

import numpy as np
import PyKDL as kdl


class Clik:
    """
        A class for Closed-Loop Inverse Kinematics controller.

        Attributes
        ----------
        kp : float
            Position feedback gain of the closed-loop controller
        n : int
            Number of joints of controlled robot
        o_ctrl : bool
            Setting if only cartesian position is controlled (False) or if orientation is also controlled (True)
        ee_vel_max : float or list of floats
            Maximum velocity of end-effector (default is infinity)
        q_vel_max : float or list of floats
            Maximum velocity of robot joints (default is infinity)
        jacsolver: kdl Jacobian solver
        fksolverpos: kdl forward kinematics solver

        Methods
        -------
        forward_kinematics(robot_joint_positions)
            Computes robot end-effector pose for the given robot joint positions
        jacobian(robot_joint_positions)
            Computes positional and orientational Jacobian for the given robot joint positions
        compute_ctrl(target_state, robot_joint_positions)
            Computes the control command of the CLIK controller
        """

    def __init__(self, chain, orientation_ctrl, kp_pos, kp_or, ee_vel_max=np.inf, q_vel_max=np.inf):
        """
        Parameters
        ----------
        chain : kdl kinematic chain
            Kinematic chain of the controlled robot
        kp : float
            Position feedback gain of the closed-loop controller
        orientation_ctrl : bool
            Setting if only cartesian position is controlled (False) or if orientation is also controlled (True)
        ee_vel_max : float or list of floats, optional
            Maximum velocity of end-effector (default is infinity)
        q_vel_max : float or list of floats, optional
            Maximum velocity of joints (default is infinity)
        """
        self.kp_p = kp_pos
        self.kp_o = kp_or
        self.n = chain.getNrOfJoints()
        self.o_ctrl = orientation_ctrl
        vel_dim = 6 if self.o_ctrl else 3
        if isinstance(ee_vel_max, float) or (isinstance(ee_vel_max, list) and len(ee_vel_max) == vel_dim):
            self.ee_vel_max = np.array(ee_vel_max)
        if isinstance(q_vel_max, float) or (isinstance(q_vel_max, list) and len(q_vel_max) == self.n):
            self.q_vel_max = np.array(q_vel_max)
        # Initialize kinematic solvers
        self.jacsolver = kdl.ChainJntToJacSolver(chain)
        self.fksolverpos = kdl.ChainFkSolverPos_recursive(chain)

    def forward_kinematics(self, robot_joint_positions):
        """Computes robot end-effector pose for the given robot joint positions.

        Parameters
        ----------
        robot_joint_positions : list
            List of current joint positions of the robot

        Returns
        -------
        list,list
            List containing end-effector position and list containing end-effector orientation given as Quaternion

        Raises
        ------
        Exception
            If the list robot_joint_positions is not of length self.n, an exception is raised
        """
        # Check length of robot_joint_positions
        if len(robot_joint_positions) != self.n:
            raise Exception("robot_joint_positions must be of size self.n, i.e. " + str(self.n))

        # Create kdl joint array
        q = kdl.JntArray(self.n)
        for i in range(self.n):
            q[i] = robot_joint_positions[i]

        # Calculate End-Effector position
        robot_ee_orientation = [0] * 4
        kdl_ee_frame = kdl.Frame()
        self.fksolverpos.JntToCart(q, kdl_ee_frame)
        [robot_ee_orientation[1], robot_ee_orientation[2], robot_ee_orientation[3], robot_ee_orientation[0]] = kdl_ee_frame.M.GetQuaternion()
        robot_ee_position = [kdl_ee_frame.p[0], kdl_ee_frame.p[1], kdl_ee_frame.p[2]]
        return robot_ee_position, robot_ee_orientation

    def jacobian(self, robot_joint_positions):
        """Computes geometrical Jacobian for the given robot joint positions.

        Parameters
        ----------
        robot_joint_positions : list
            List of current joint positions of the robot

        Returns
        -------
        2D list
            2D list representation of the Jacobian

        Raises
        ------
        Exception
            If the list robot_joint_positions is not of length self.n, an exception is raised
        """
        # Check length of robot_joint_positions
        if len(robot_joint_positions) != self.n:
            raise Exception("robot_joint_positions must be of size self.n, i.e. " + str(self.n))

        # Create kdl joint array
        q = kdl.JntArray(self.n)
        for i in range(self.n):
            q[i] = robot_joint_positions[i]

        # Calculate Jacobian
        jac_kdl = kdl.Jacobian(self.n)
        self.jacsolver.JntToJac(q, jac_kdl)
        jac = np.zeros((6, self.n))
        for i in range(6):
            for j in range(self.n):
                jac[i, j] = jac_kdl[i, j]

        return jac.tolist()

    @staticmethod
    def skew(v):
        return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])

    def compute_ctrl(self, target_state, robot_joint_positions):
        """Computes the control command of the CLIK controller.

        If the list robot_joint_positions is not of length self.n, a list of zeros is returned.
        If the list corresponding to key 'position' in target_state is not of length 3/7 (o_ctrl=False/True),
        the feedback part of the controller is set to zero.
        If the list corresponding to key 'velocity' in target_state is not of length 3/6 (o_ctrl=False/True),
        the feed-forward part of the controller is set to zero.

        Parameters
        ----------
        target_state : dict with keys 'position' and 'velocity'
            The sound the animal makes (default is None)
        robot_joint_positions : list
            List of current joint positions of the robot

        Returns
        -------
        list
            List of velocity commands for the robot joints
        """
        # Get Jacobian and current end-effector pose
        try:
            jac = np.array(self.jacobian(robot_joint_positions))
            [robot_ee_position, robot_ee_orientation] = self.forward_kinematics(robot_joint_positions)
            robot_ee_position = np.array(robot_ee_position)
            robot_ee_orientation = np.array(robot_ee_orientation)
        except Exception:
            return [0] * self.n

        # Extract target
        target_pos = np.array(target_state["position"])
        target_vel = np.array(target_state["velocity"])

        # Calculate Cartesian control
        dim_pos = 7 if self.o_ctrl else 3
        dim_vel = 6 if self.o_ctrl else 3
        ff = np.zeros(dim_vel)
        fb = np.zeros(dim_vel)
        if target_vel.size == dim_vel:
            ff = target_vel
        if target_pos.size == dim_pos:
            pos_error = target_pos[0:3] - robot_ee_position
            fb_p = self.kp_p * pos_error
            if self.o_ctrl:
                target_orientation = target_pos[3:7] / np.linalg.norm(target_pos[3:7])
                or_error = robot_ee_orientation[0] * target_orientation[1:4] - target_orientation[0] * robot_ee_orientation[1:4] - self.skew(target_orientation[1:4]).dot(robot_ee_orientation[1:4])
                fb_o = self.kp_o * or_error
            else:
                fb_o = np.array([])
            fb = np.concatenate((fb_p, fb_o))

        if not self.o_ctrl:
            jac = jac[0:3, :]

        # Saturate Cartesian velocity
        ee_vel_cmd = np.clip(ff + fb, -self.ee_vel_max, self.ee_vel_max)
        # Perform inverse kinematics
        jac_pinv = np.linalg.pinv(jac)
        qd_cmd = np.dot(jac_pinv, ee_vel_cmd)
        # Saturate joint velocity
        qd_cmd = np.clip(qd_cmd, -self.q_vel_max, self.q_vel_max)

        return qd_cmd.tolist()
