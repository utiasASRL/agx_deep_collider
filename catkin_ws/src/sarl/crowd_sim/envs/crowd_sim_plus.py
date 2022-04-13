from crowd_sim.envs import CrowdSim
import logging
import gym
import matplotlib.lines as mlines
import numpy as np
# import rvo2
from matplotlib import patches
from numpy.linalg import norm
from crowd_sim.envs.utils.human import Human
from crowd_sim.envs.utils.info import *
from crowd_sim.envs.utils.utils import point_to_segment_dist


# Something that would belong in utils.info
class Frozen(object):
    def __init__(self):
        pass

    def __str__(self):
        return 'Frozen'

class CrowdSimPlus(CrowdSim):

    def __init__(self):
        super().__init__()
        # additional configurations
        # reward function
        self.freezing_penalty = None
        # simulation configuration
        self.config = None
        self.rect_width = None
        self.rect_height = None

    def configure(self, config):
        print('hello')
        print(config)
        super().configure(config)
        self.freezing_penalty = config.getfloat('reward', 'freezing_penalty')
        # self.freezing_penalty = -0.125
        if self.config.get('humans', 'policy') == 'orca':
            self.rect_width = config.getfloat('sim', 'rect_width')
            self.rect_height = config.getfloat('sim', 'rect_height')
        else:
            raise NotImplementedError
        logging.info('Rect width: {}, rect height: {}'.format(self.rect_width, self.rect_height))

    def generate_random_human_position(self, human_num, rule):
        """
        Generate human position according to certain rule, extend CrowdSim to include more rules.
        Rule square_crossing: generate start/goal position at two sides of y-axis
        Rule circle_crossing: generate start position on a circle, goal position is at the opposite side

        :param human_num:
        :param rule:
        :return:
        """
        # initial min separation distance to avoid danger penalty at beginning
        if rule == 'square_crossing':
            self.humans = []
            for i in range(human_num):
                self.humans.append(self.generate_square_crossing_human())
        elif rule == 'circle_crossing':
            self.humans = []
            for i in range(human_num):
                self.humans.append(self.generate_circle_crossing_human())
        elif rule == 'hallway':
            self.humans = []
            for i in range(human_num):
                self.humans.append(self.generate_hallway_human())
        elif rule == 'mixed':
            # mix different raining simulation with certain distribution
            static_human_num = {0: 0.05, 1: 0.2, 2: 0.2, 3: 0.3, 4: 0.1, 5: 0.15}
            dynamic_human_num = {1: 0.3, 2: 0.3, 3: 0.2, 4: 0.1, 5: 0.1}
            static = True if np.random.random() < 0.2 else False
            prob = np.random.random()
            for key, value in sorted(static_human_num.items() if static else dynamic_human_num.items()):
                if prob - value <= 0:
                    human_num = key
                    break
                else:
                    prob -= value
            self.human_num = human_num
            self.humans = []
            if static:
                # randomly initialize static objects in a square of (width, height)
                width = 4
                height = 8
                if human_num == 0:
                    human = Human(self.config, 'humans')
                    human.set(0, -10, 0, -10, 0, 0, 0)
                    self.humans.append(human)
                for i in range(human_num):
                    human = Human(self.config, 'humans')
                    if np.random.random() > 0.5:
                        sign = -1
                    else:
                        sign = 1
                    while True:
                        px = np.random.random() * width * 0.5 * sign
                        py = (np.random.random() - 0.5) * height
                        collide = False
                        for agent in [self.robot] + self.humans:
                            if norm((
                                    px - agent.px, py - agent.py)) < human.radius + agent.radius + self.discomfort_dist:
                                collide = True
                                break
                        if not collide:
                            break
                    human.set(px, py, px, py, 0, 0, 0)
                    self.humans.append(human)
            else:
                # the first 2 two humans will be in the circle crossing scenarios
                # the rest humans will have a random starting and end position
                for i in range(human_num):
                    if i < 2:
                        human = self.generate_circle_crossing_human()
                    else:
                        human = self.generate_square_crossing_human()
                    self.humans.append(human)
        else:
            raise ValueError("Rule doesn't exist")

    def generate_hallway_human(self):
        # IPR right now same as square
        human = Human(self.config, 'humans')
        if self.randomize_attributes:
            # if sampling random attributes, only set v_pref randomly, not radius
            human.v_pref = np.random.uniform(0.5, 1.5)

        # Decide whether the human is travelling up or down (y dir)
        if np.random.random() > 0.5:
            dir_sign = 1  # walk up
        else:
            dir_sign = -1  # walk down

        # Decide whether the human is walking on the left or on the right (x dir)
        prob_right = 0.8
        if dir_sign > 0:
            right_num = prob_right
        else:
            right_num = 1 - prob_right

        # whether the human is walking on the right or on the left
        if np.random.random() < right_num:
            wor_sign = -1 # walk on the right (perspective of y:up)
        else:
            wor_sign = 1 # walk on the left (perspective of y:up)
                     # i.e. walk on the right (perspective of y:down)

        prob_cross = 0.3
        # if the human is on the left, switch with high probability, else don't
        if np.random.random() < right_num:
            prob_cross = 1 - prob_cross

        # whether the human crosses right to left or vice versa or not
        if np.random.random() < prob_cross:
            cross_sign = -wor_sign
        else:
            cross_sign = wor_sign

        while True:
            px = (np.random.random()) * 0.5 * wor_sign * self.rect_width
            py = (np.random.random()) * 0.5 * dir_sign * self.rect_height
            collide = False
            for agent in [self.robot] + self.humans:
                if norm((px - agent.px, py - agent.py)) < human.radius + agent.radius + self.discomfort_dist:
                    collide = True
                    break
            if not collide:
                break

        while True:
            gx = (np.random.random()) * 0.5 * cross_sign * self.rect_width
            gy = (np.random.random()) * 0.5 * -dir_sign * self.rect_height + -dir_sign * self.rect_height
            collide = False
            for agent in [self.robot] + self.humans:
                if norm((gx - agent.gx, gy - agent.gy)) < human.radius + agent.radius + self.discomfort_dist:
                    collide = True
                    break
            if not collide:
                break
        human.set(px, py, gx, gy, 0, 0, 0)
        return human

    def step(self, action, update=True):
        """
        Compute actions for all agents, detect collision, update environment and return (ob, reward, done, info)
        Extend CrowdSim to (1) account for robot freezing and (2) continue towards the goal after a collision.

        """
        human_actions = []
        for human in self.humans:
            # observation for humans is always coordinates
            ob = [other_human.get_observable_state() for other_human in self.humans if other_human != human]
            if self.robot.visible:
                ob += [self.robot.get_observable_state()]
            human_actions.append(human.act(ob))

        # collision detection
        dmin = float('inf')
        collision = False
        for i, human in enumerate(self.humans):
            px = human.px - self.robot.px
            py = human.py - self.robot.py
            if self.robot.kinematics == 'holonomic':
                vx = human.vx - action.vx
                vy = human.vy - action.vy
            else:
                vx = human.vx - action.v * np.cos(action.r + self.robot.theta)
                vy = human.vy - action.v * np.sin(action.r + self.robot.theta)
            ex = px + vx * self.time_step
            ey = py + vy * self.time_step
            # closest distance between boundaries of two agents
            closest_dist = point_to_segment_dist(px, py, ex, ey, 0, 0) - human.radius - self.robot.radius
            if closest_dist < 0:
                collision = True
                # logging.debug("Collision: distance between robot and p{} is {:.2E}".format(i, closest_dist))
                break
            elif closest_dist < dmin:
                dmin = closest_dist

        # collision detection between humans
        human_num = len(self.humans)
        for i in range(human_num):
            for j in range(i + 1, human_num):
                dx = self.humans[i].px - self.humans[j].px
                dy = self.humans[i].py - self.humans[j].py
                dist = (dx ** 2 + dy ** 2) ** (1 / 2) - self.humans[i].radius - self.humans[j].radius
                if dist < 0:
                    # detect collision but don't take humans' collision into account
                    logging.debug('Collision happens between humans in step()')

        # check if the robot has frozen
        frozen_robot = False
        if self.robot.kinematics == 'holonomic':
            frozen_robot = action.vx < 0.005 and action.vy < 0.005
        else:
            frozen_robot = action.v < 0.005


        # check if reaching the goal
        end_position = np.array(self.robot.compute_position(action, self.time_step))
        reached_goal = norm(end_position - np.array(self.robot.get_goal_position())) < self.robot.radius

        if reached_goal:
            reward = self.success_reward
            done = True
            info = ReachGoal()
        elif self.global_time >= self.time_limit - 1:
            reward = 0
            done = True
            info = Timeout()
        elif collision:
            reward = self.collision_penalty
            done = False
            info = Collision()
            logging.debug('Collision in step()')
        elif frozen_robot:
            reward = self.freezing_penalty
            done = False
            info = Frozen()
            logging.debug('Freezing robot in step()')
        elif dmin < self.discomfort_dist:
            # only penalize agent for getting too close if it's visible
            # adjust the reward based on FPS
            reward = (dmin - self.discomfort_dist) * self.discomfort_penalty_factor * self.time_step
            done = False
            info = Danger(dmin)
        else:
            reward = 0
            done = False
            info = Nothing()

        if update:
            # store state, action value and attention weights
            self.states.append([self.robot.get_full_state(), [human.get_full_state() for human in self.humans]])
            if hasattr(self.robot.policy, 'action_values'):
                self.action_values.append(self.robot.policy.action_values)
            if hasattr(self.robot.policy, 'get_attention_weights'):
                self.attention_weights.append(self.robot.policy.get_attention_weights())

            # update all agents
            self.robot.step(action)
            for i, human_action in enumerate(human_actions):
                self.humans[i].step(human_action)
            self.global_time += self.time_step
            for i, human in enumerate(self.humans):
                # only record the first time the human reaches the goal
                if self.human_times[i] == 0 and human.reached_destination():
                    self.human_times[i] = self.global_time

            # compute the observation
            if self.robot.sensor == 'coordinates':
                ob = [human.get_observable_state() for human in self.humans]
            elif self.robot.sensor == 'RGB':
                raise NotImplementedError
        else:
            if self.robot.sensor == 'coordinates':
                ob = [human.get_next_observable_state(action) for human, action in zip(self.humans, human_actions)]
            elif self.robot.sensor == 'RGB':
                raise NotImplementedError

        return ob, reward, done, info