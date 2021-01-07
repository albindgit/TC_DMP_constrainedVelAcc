#!/usr/bin/env python

import rospy
from robot_ctrl.msg import CtrlTarget
from std_msgs.msg import String, Empty
import time

rospy.init_node('experiment_initializer_node', anonymous=True)

pub_pos_cmd = rospy.Publisher('robot_interface/pos_command', CtrlTarget, queue_size=1)
pub_rc = rospy.Publisher('robot_ctrl/reload_params', Empty, queue_size=1)
pub_mp_cmd = rospy.Publisher('motion_planning/command', String, queue_size=1)

time.sleep(0.5)

# Reload parameters
pub_rc.publish()
pub_mp_cmd.publish("load_params")

time.sleep(0.5)

pub_mp_cmd.publish("reset")

if not rospy.has_param("motion_planning/init_pos"):
    rospy.logerr("Parameter motion_planning/init_pos not set.")
    exit(1)

time.sleep(0.5)

# Move robot to init pos
t = 0.
msg = CtrlTarget()
msg.position = rospy.get_param("motion_planning/init_pos")
while t < 5.:
    msg.header.stamp = rospy.Time.now()
    pub_pos_cmd.publish(msg)
    t += 0.1
    time.sleep(0.1)
