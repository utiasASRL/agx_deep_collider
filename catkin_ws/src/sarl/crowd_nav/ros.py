import rospy
import os
import configparser
import numpy as np
import torch
# import gym
import math
import tf
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from crowd_nav.utils.explorer import Explorer
from crowd_nav.policy.policy_factory import policy_factory
from crowd_sim.envs.utils.robot import Robot
from crowd_sim.envs.policy.orca import ORCA
from crowd_sim.envs.utils.state import ObservableState, JointState
from zeus_msgs.msg import Detections3D, BoundingBox3D
from mutils import *

class SARLNode:

    def __init__(self):
        rospy.init_node('sarl_node')
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
        
        self.policy = policy_factory['sarl']()
        policy_config = configparser.RawConfigParser()
        policy_config_file = 'configs/policy.config'
        policy_config.read(policy_config_file)
        self.policy.configure(policy_config)
        self.policy.query_env = False
        self.policy.set_phase('test')
        self.policy.set_device('cpu')
        self.policy.time_step = 0.1
        self.dvmax = 0.5 * self.policy.time_step
        self.dwmax = 0.4 * self.policy.time_step
        self.wmax = 0.8
        self.policy.v_pref = 0.4

        env_config = configparser.RawConfigParser()
        env_config_file = 'configs/env.config'
        env_config.read(env_config_file)
        
        # env_config.read(env_config_file)
        # env = gym.make('CrowdSim-v0')
        # env.configure(env_config)

        self.robot = Robot(env_config, 'robot')
        self.robot.set_policy(self.policy)
        self.robot.time_step = self.policy.time_step
        self.robot.v_pref = self.policy.v_pref
        # env.set_robot(robot)

        # policy.set_env(env)
        # robot.print_info()




    ## TODO: set goal somehow? (with rviz maybe, visualize in RVIZ)

    def callback(self, msg):
        print('updated human states received')
        gx, gy = self.robot.get_goal_position()
        if gx is None or gy is None:
            return
        vx = np.cos(self.yaw) * self.body_vel
        vy = np.sin(self.yaw) * self.body_vel
        self.robot.set_velocity([vx, vy])
        self.robot.policy.rebuild_action_space(self.policy.v_pref, self.body_vel,
            self.ang_vel, self.dvmax, self.dwmax, self.wmax, self.policy.time_step)
        ob = []
        for bb in msg.bbs:
            r = (bb.l + bb.w) / 2
            obs = ObservableState(bb.x, bb.y, bb.x_dot, bb.y_dot, r)
            ob.append(obs)
        action = self.robot.act(ob)
        print(action)
        tmsg = Twist()
        tmsg.twist.twist.linear.x = action.v
        tmsg.twist.twist.angular.z = action.r
        self.cmd_pub.publish(tmsg)

    def goal_callback(self, msg):
        # TODO: update the robot's goal position
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
    sarl = SARLNode()
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        sarl.update_transforms()
        rate.sleep()
