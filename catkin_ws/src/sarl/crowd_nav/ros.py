import rospy
import os
import numpy as np
import torch
import gym
import math
import tf
import geometry_msgs.msg
from crowd_nav.utils.explorer import Explorer
from crowd_nav.policy.policy_factory import policy_factory
from crowd_sim.envs.utils.robot import Robot
from crowd_sim.envs.policy.orca import ORCA
from zeus_msgs.msg import Detections3D, BoundingBox3D
from mutils import *

class SARLNode:

    def __init__(self):
        rospy.init_node('sarl_node')
        rospy.Subscriber('/Object/Detections3D', Detections3D, self.callback)
        model_weights = 'model/resumed_rl_model.pth'
        self.pos = [0, 0, 0]
        self.T_mv = np.identity(4)
        self.listener = tf.TransformListener()
        self.yaw = 0
        
        self.policy = policy_factory['sarl']()
        policy_config = configparser.RawConfigParser()
        policy_config_file = 'configs/policy.config'
        policy_config.read(policy_config_file)
        self.policy.configure(policy_config)
        self.policy.query_env = False

        env_config_file = 'config/env.config'
        env_config = configparser.RawConfigParser()
        # env_config.read(env_config_file)
        # env = gym.make('CrowdSim-v0')
        # env.configure(env_config)

        robot = Robot(env_config, 'robot')
        robot.set_policy(policy)
        env.set_robot(robot)

        # policy.set_env(env)
        # robot.print_info()

        # self.policy.set_phase('test')
        # self.policy.set_device('cpu')


    ## TODO: set goal somehow? (with rviz maybe, visualize in RVIZ)
    ## TODO: set action space appropriately

    def callback(self, msg):
        print(msg)
        # TODO: create ob from msg
        action = robot.act(ob)
        # TODO: convert action to what robot can actually do


    def update_transforms(self):
        try:
            self.pos, rot = self.listener.lookupTransform("/map", "/velodyne", rospy.Time(0))
            self.T_mv = np.identity(4)
            self.T_mv[:3, :3] = quaternionToRot(np.array(rot)).transpose()
            self.T_mv[:3, 3:] = np.array(self.pos).reshape(3, 1)
            self.yaw, _, _ = rotToYawPitchRoll(self.T_mv[:3, :3])

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return


if __name__ == '__main__':
    sarl = SARLNode()
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        sarl.update_transforms()
        rate.sleep()
