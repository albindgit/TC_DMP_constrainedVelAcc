#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty
from robot_ctrl.msg import CtrlTarget

class VelFFPosFB:
    def __init__(self, kp, vel_max):
        # Controller parameters:
        self.kp = kp
        self.vel_max = vel_max
        # Command state
        self.vel_cmd = None

    def compute_ctrl(self, target_state, robot_state):
        ff = 0.
        fb = 0.
        target_position = np.array(target_state.position)
        target_velocity = np.array(target_state.velocity)
        robot_position = np.array(robot_state.position)
        if target_velocity.size > 0:
            ff = target_velocity
        if target_position.size > 0:
            fb = self.kp * (target_position - robot_position)
        vel_cmd = ff + fb
        np.clip(vel_cmd, -self.vel_max, self.vel_max, out=vel_cmd)

        return vel_cmd


class VelFFPosFBNode:
    def __init__(self):
        self.ctrl = None
        self.rate = None
        self.max_msg_deadtime = 0.1
        self.robot_state = CtrlTarget()
        self.target_state = CtrlTarget()

        if not self.load_params(0):
            exit(1)

        self.vel_cmd_publisher = rospy.Publisher("robot_interface/vel_command", CtrlTarget, queue_size=1)
        rospy.Subscriber("robot_interface/joint_states", JointState, self.state_update)
        rospy.Subscriber("robot_ctrl/command", CtrlTarget, self.target_update)
        rospy.Subscriber("robot_ctrl/reload_params", Empty, self.load_params)

        self.spin()

    def load_params(self,dummydata):
        # Check necessary parameter existences
        if not rospy.has_param('robot_ctrl/update_rate'):
            rospy.logerr("Parameter " + rospy.get_namespace() + "robot_ctrl/update_rate not set.")
            return False
        if not rospy.has_param('robot_ctrl/kp'):
            rospy.logerr("Parameter " + rospy.get_namespace() + "robot_ctrl/kp not set.")
            return False
        if not rospy.has_param('robot_ctrl/vel_max'):
            rospy.logerr("Parameter " + rospy.get_namespace() + "robot_ctrl/vel_max not set.")
            return False
        # Load parameters from server
        self.rate = rospy.Rate(rospy.get_param('robot_ctrl/update_rate'))
        kp = rospy.get_param('robot_ctrl/kp')
        vel_max = np.array(rospy.get_param('robot_ctrl/vel_max'))
        # Create control object
        self.ctrl = VelFFPosFB(kp, vel_max)
        rospy.loginfo("Loaded robot_ctrl parameters.")
        return True

    def state_update(self, msg):
        self.robot_state = msg

    def target_update(self, msg):
        self.target_state = msg
        self.target_state.header.stamp = rospy.Time.now()

    def spin(self):
        while (not rospy.is_shutdown()):
            target_msg_deadtime = (rospy.Time.now() - self.target_state.header.stamp).to_sec()
            robot_msg_deadtime = (rospy.Time.now() - self.robot_state.header.stamp).to_sec()
            if target_msg_deadtime < self.max_msg_deadtime and robot_msg_deadtime < self.max_msg_deadtime:
                # Compute velocity command
                vel_cmd = self.ctrl.compute_ctrl(self.target_state, self.robot_state)

                # publish joint velocities command
                cmd_msg = CtrlTarget()
                cmd_msg.header.stamp = rospy.Time.now()
                cmd_msg.velocity = vel_cmd
                self.vel_cmd_publisher.publish(cmd_msg)

            self.rate.sleep()


# Here is the main entry point
if __name__ == '__main__':
    try:
        # Init the node
        rospy.init_node('VelFFPosFBNode')

        # Initializing and continue running the main class
        VelFFPosFBNode()

    except rospy.ROSInterruptException:
        pass