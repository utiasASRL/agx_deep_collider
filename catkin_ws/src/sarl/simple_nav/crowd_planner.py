#!/usr/bin/env python3
import os
import rospy
import configparser
import numpy as np
import torch
import math
import tf
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from simple_nav.policy.policy_factory import policy_factory
from crowd_sim.envs.utils.robot import Robot
from crowd_sim.envs.policy.orca import ORCA
from crowd_sim.envs.utils.state import ObservableState, JointState
from zeus_msgs.msg import Detections3D, BoundingBox3D
from mutils import *

pkg_name = "crowd_planner"

class CrowdNavNode:

    def __init__(self):
        rospy.init_node('crowd_planner')
        rospy.Subscriber('/Object/Detections3D', Detections3D, self.callback)
        rospy.Subscriber('/move_base/current_goal', PoseStamped, self.goal_callback)
        rospy.Subscriber('/jackal_velocity_controller/odom', Odometry, self.odom_callback)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        model_weights = 'model/resumed_rl_model.pth'
        self.pos = [0, 0, 0]
        self.T_mv = np.identity(4)
        self.listener = tf.TransformListener()
        self.yaw = 0
        self.body_vel = 0
        self.ang_vel = 0

        policy_config = configparser.RawConfigParser()
        policy_config_file = 'configs/policy.config'
        if not os.path.exists(policy_config_file):
            policy_config_file = rospy.get_param(pkg_name + "/policy_config")
        policy_config.read(policy_config_file)
        env_config = configparser.RawConfigParser()
        env_config_file = 'configs/env.config'
        if not os.path.exists(env_config_file):
            env_config_file = rospy.get_param(pkg_name + "/env_config")
        env_config.read(env_config_file)

        self.policy_name = env_config.get('robot', 'policy')
        self.policy = policy_factory[self.policy_name]()
        self.policy.configure(policy_config)
        if self.policy.trainable:
            if not os.path.exists(model_weights):
                model_weights = rospy.get_param(pkg_name + "/model_weights")
            self.policy.get_model().load_state_dict(torch.load(model_weights))
        self.policy.set_phase('test')
        self.policy.set_device('cpu')

        self.robot = Robot(env_config, 'robot')
        self.robot.set_policy(self.policy)
        self.robot.time_step = env_config.getfloat('env', 'time_step')
        self.policy.v_pref = self.robot.v_pref
        self.policy.time_step = self.robot.time_step

        if self.policy_name == 'dwa':
            self.policy.configure_dwa(env_config, 'robot')

        self.debug = env_config.getboolean('robot', 'debug')
        self.acc_max = env_config.getfloat('robot', 'acc_max')
        self.ang_acc_max = env_config.getfloat('robot', 'ang_acc_max')
        self.wmax = env_config.getfloat('robot', 'wmax')
        self.dvmax = self.acc_max * self.policy.time_step
        self.dwmax = self.ang_acc_max * self.policy.time_step
        self.inflate_radius = env_config.getfloat('robot', 'inflate_radius')
        self.min_radius = env_config.getfloat('robot', 'min_radius')

    def callback(self, msg):
        gx, gy = self.robot.get_goal_position()
        if gx is None or gy is None:
            return
        vx = np.cos(self.yaw) * self.body_vel
        vy = np.sin(self.yaw) * self.body_vel
        self.robot.set_velocity([vx, vy])
        if self.policy_name == 'sarl':
            self.robot.policy.rebuild_action_space(self.policy.v_pref, self.body_vel,
                self.ang_vel, self.dvmax, self.dwmax, self.wmax,
                self.policy.time_step, self.debug)
        ob = []
        for bb in msg.bbs:
            r = (bb.l + bb.w) / 2 + self.inflate_radius
            if r < self.min_radius:
                r = self.min_radius
            obs = ObservableState(bb.x, bb.y, bb.x_dot, bb.y_dot, r)
            ob.append(obs)
        action = self.robot.act(ob)
        if self.debug:
            print(action)
        tmsg = Twist()
        tmsg.linear.x = action.v
        tmsg.angular.z = action.r / self.policy.time_step
        self.cmd_pub.publish(tmsg)

    def goal_callback(self, msg):
        if self.debug:
            print('goal position: {} {}'.format(msg.pose.position.x, msg.pose.position.y))
        self.robot.set_goal_position([msg.pose.position.x, msg.pose.position.y])

    def odom_callback(self, msg):
        self.body_vel = msg.twist.twist.linear.x
        self.ang_vel = msg.twist.twist.angular.z

    def update_transforms(self):
        try:
            self.pos, rot = self.listener.lookupTransform("/map", "/velodyne", rospy.Time(0))
            self.T_mv = np.identity(4)
            self.T_mv[:3, :3] = quaternionToRot(np.array(rot)).transpose()
            self.T_mv[:3, 3:] = np.array(self.pos).reshape(3, 1)
            self.yaw, _, _ = rotToYawPitchRoll(self.T_mv[:3, :3])
            self.yaw *= -1
            self.robot.set_position(self.pos)
            self.robot.set_orientation(self.yaw)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return


if __name__ == '__main__':
    sarl = CrowdNavNode()
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        sarl.update_transforms()
        rate.sleep()
