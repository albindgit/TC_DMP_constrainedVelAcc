#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Empty
import time
import numpy as np

rospy.init_node('experiment_initializer_node', anonymous=True)

pub_pos_cmd = rospy.Publisher('robot_interface/joint_pos_command', JointState, queue_size=1)
pub_rc_reload = rospy.Publisher('robot_ctrl/reload_params', Empty, queue_size=1)
pub_mp_cmd = rospy.Publisher('motion_planning/command', String, queue_size=1)
pub_exp_step = rospy.Publisher('experiment_step', JointState, queue_size=1)


time.sleep(0.5)

# Reload parameters
pub_rc_reload.publish()
pub_mp_cmd.publish("load_params")

time.sleep(0.5)

# Reset motion planner
pub_mp_cmd.publish("reset")

time.sleep(0.5)


# Load calibration positions
if rospy.has_param('calibration_loadpath'):
    calibration_loadpath = rospy.get_param('calibration_loadpath')
    cal_pos = np.genfromtxt(calibration_loadpath, delimiter=',')
else:
    rospy.logerr("Could not load calibration points. Parameter " + rospy.get_namespace() +
                 "calibration_loadpath not found")

t0 = rospy.Time.now()
t_stamp = rospy.Time.now()
t_cal = 3.0
t = 0
sleep_time = 0

msg = JointState()
# Move to calibration positions
msg = JointState()
rospy.loginfo("Calibration start")
for cal_pos_i in cal_pos:
    msg.position = cal_pos_i
    pub_pos_cmd.publish(msg)

    msg.header.stamp = rospy.Time.now()
    msg.position = [0]
    pub_exp_step.publish(msg)

    t = t + t_cal
    t_stamp = rospy.Time.now()
    sleep_time = t0 + rospy.Duration(t) - t_stamp
    time.sleep(sleep_time.to_sec())


msg = JointState()
msg.header.stamp = rospy.Time.now()
msg.position = [1]
pub_exp_step.publish(msg)
# Move robot to init position
if not rospy.has_param("motion_planning/init_pos"):
    rospy.logerr("Parameter motion_planning/init_pos not set.")
    exit(1)
msg = JointState()
msg.position = rospy.get_param("motion_planning/init_pos")
pub_pos_cmd.publish(msg)

t = t + 4.0
t_stamp = rospy.Time.now()
sleep_time = t0 + rospy.Duration(t) - t_stamp
time.sleep(sleep_time.to_sec())

msg = JointState()
msg.header.stamp = rospy.Time.now()
msg.position = [2]
pub_exp_step.publish(msg)
# Start experiment
rospy.loginfo("Experiment start")
pub_mp_cmd.publish("start")
