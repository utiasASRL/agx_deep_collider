import logging
import argparse
import configparser
import os
import torch
import numpy as np
import gym
from simple_nav.utils.explorer import ExplorerPlus
from policy.policy_factory import policy_factory
from crowd_sim.envs.utils.robot import Robot
from crowd_sim.envs.policy.orca import ORCA

# such that animations work properly in Pycharm.
# import matplotlib
# matplotlib.use("TkAgg")

def main():
    parser = argparse.ArgumentParser('Parse configuration file')
    parser.add_argument('--env_config', type=str, default='configs/env.config')
    parser.add_argument('--policy_config', type=str, default='configs/policy.config')
    parser.add_argument('--policy', type=str, default=None)
    parser.add_argument('--model_dir', type=str, default=None)
    parser.add_argument('--il', default=False, action='store_true')
    parser.add_argument('--gpu', default=False, action='store_true')
    parser.add_argument('--visualize', default=False, action='store_true')
    parser.add_argument('--phase', type=str, default='test')
    parser.add_argument('--test_case', type=int, default=None)
    parser.add_argument('--square', default=False, action='store_true')
    parser.add_argument('--circle', default=False, action='store_true')
    parser.add_argument('--hallway', default=False, action='store_true')
    parser.add_argument('--video_file', type=str, default=None)
    parser.add_argument('--traj', default=False, action='store_true')
    args = parser.parse_args()

    if args.model_dir is not None:
        env_config_file = os.path.join(args.model_dir, os.path.basename(args.env_config))
        policy_config_file = os.path.join(args.model_dir, os.path.basename(args.policy_config))
        if args.il:
            model_weights = os.path.join(args.model_dir, 'il_model.pth')
        else:
            if os.path.exists(os.path.join(args.model_dir, 'resumed_rl_model.pth')):
                model_weights = os.path.join(args.model_dir, 'resumed_rl_model.pth')
            else:
                model_weights = os.path.join(args.model_dir, 'rl_model.pth')
    else:
        env_config_file = args.env_config
        policy_config_file = args.env_config

    # configure logging and device
    logging.basicConfig(level=logging.INFO, format='%(asctime)s, %(levelname)s: %(message)s',
                        datefmt="%Y-%m-%d %H:%M:%S")
    device = torch.device("cuda:0" if torch.cuda.is_available() and args.gpu else "cpu")
    logging.info('Using device: %s', device)

    # configure environment
    env_config = configparser.RawConfigParser()
    env_config.read(env_config_file)
    env = gym.make('CrowdSimPlus-v0')
    print(args.env_config)
    print(args.policy_config)
    env.configure(env_config)
    if args.square:
        env.test_sim = 'square_crossing'
    elif args.circle:
        env.test_sim = 'circle_crossing'
    elif args.hallway:
        env.test_sim = 'hallway'
    else:
        env.test_sim = 'hallway'

    # configure policy
    if args.policy is not None:
        policy = policy_factory[args.policy]()
    else:
        policy = policy_factory[env_config.get('robot', 'policy')]()
    policy_config = configparser.RawConfigParser()
    policy_config.read(policy_config_file)
    policy.configure(policy_config)
    if policy.trainable:
        if args.model_dir is None:
            parser.error('Trainable policy must be specified with a model weights directory')
        policy.get_model().load_state_dict(torch.load(model_weights))


    # make robot and set robot in environment
    robot = Robot(env_config, 'robot')
    robot.set_policy(policy)
    env.set_robot(robot)

    # make explorer
    explorer = ExplorerPlus(env, robot, device, gamma=0.9)

    # set policy details
    policy.set_phase(args.phase)
    policy.set_device(device)
    # set safety space for ORCA in non-cooperative simulation
    if isinstance(robot.policy, ORCA):
        if robot.visible:
            robot.policy.safety_space = 0
        else:
            # because invisible case breaks the reciprocal assumption
            # adding some safety space improves ORCA performance. Tune this value based on your need.
            robot.policy.safety_space = 0
        logging.info('ORCA agent buffer: %f', robot.policy.safety_space)

    # set environment in policy
    policy.set_env(env)
    robot.print_info()
    if args.visualize:
        if args.test_case is None:
            viz_test_case = np.random.choice(env.case_capacity[args.phase])
        else:
            viz_test_case = args.test_case
        logging.info('About to start visualizing phase: %s, test case: %i.', args.phase, viz_test_case)
        ob = env.reset(args.phase, viz_test_case)
        done = False
        last_pos = np.array(robot.get_position())
        while not done:
            action = robot.act(ob)
            ob, _, done, info = env.step(action)
            current_pos = np.array(robot.get_position())
            logging.info('Displacement: %.2f', np.linalg.norm(current_pos - last_pos))
            last_pos = current_pos

        logging.info('It takes %.2f seconds to finish. Final status is %s', env.global_time, info)
        if args.traj:
            env.render('traj', args.video_file)
        else:
            env.render('video', args.video_file)


        if robot.visible and info == 'reach goal':
            human_times = env.get_human_times()
            logging.info('Average time for humans to reach goal: %.2f', sum(human_times) / len(human_times))
    else:
        explorer.run_k_episodes(env.case_size[args.phase], args.phase, print_failure=True)


if __name__ == '__main__':
    main()
