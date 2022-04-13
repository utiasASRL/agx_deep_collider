from crowd_nav.utils.explorer import *
from crowd_sim.envs.crowd_sim_plus import *

class ExplorerPlus(Explorer):

    def __init__(self, env, robot, device, memory=None, gamma=None, target_policy=None):
        super().__init__(env, robot, device, memory, gamma, target_policy)


    def run_k_episodes(self, k, phase, update_memory=False, imitation_learning=False, episode=None,
                       print_failure=False):
        self.robot.policy.set_phase(phase)
        success_times = []
        timeout_times = []
        success = 0
        num_collisions = 0
        num_frozen = 0
        num_timesteps = 0
        timeout = 0
        too_close = 0
        min_dist = []
        cumulative_rewards = []
        collision_times = []
        collision_cases = []
        frozen_times = []
        freezing_cases = []
        timeout_cases = []
        for i in range(k):
            ob = self.env.reset(phase)
            done = False
            states = []
            actions = []
            rewards = []
            had_first_collision = False
            had_first_freezing = False
            while not done:
                action = self.robot.act(ob)
                ob, reward, done, info = self.env.step(action)
                states.append(self.robot.policy.last_state)
                actions.append(action)
                rewards.append(reward)

                if isinstance(info, Danger):
                    too_close += 1
                    min_dist.append([i, info.min_dist])

                if isinstance(info, Collision):
                    num_collisions += 1
                    if not had_first_collision:
                        collision_cases.append(i)
                        had_first_collision = True
                    collision_times.append([i, self.env.global_time])
                elif isinstance(info, Frozen):
                    num_frozen += 1
                    if not had_first_freezing:
                        freezing_cases.append(i)
                        had_first_freezing = True
                    frozen_times.append([i, self.env.global_time])
                num_timesteps +=1

            if isinstance(info, ReachGoal):
                success += 1
                success_times.append(self.env.global_time)
            elif isinstance(info, Timeout):
                timeout += 1
                timeout_cases.append(i)
                timeout_times.append(self.env.time_limit)
            else:
                raise ValueError('Invalid exit signal from environment')

            if update_memory:
                if isinstance(info, ReachGoal) or had_first_collision:
                    # only add positive(success) or negative(collision) experience in experience set
                    self.update_memory(states, actions, rewards, imitation_learning)

            cumulative_rewards.append(sum([pow(self.gamma, t * self.robot.time_step * self.robot.v_pref)
                                           * reward for t, reward in enumerate(rewards)]))
            logging.info('Done scenario {:d} of {:d}'.format(i+1, k))

        success_rate = success / k
        timeout_rate = timeout / k
        assert success + timeout == k
        avg_nav_time = sum(success_times) / len(success_times) if success_times else self.env.time_limit
        collision_rate = num_collisions / num_timesteps
        frozen_rate = num_frozen / num_timesteps
        extra_info = '' if episode is None else 'in episode {} '.format(episode)
        logging.info('{:<5} {}has success rate: {:.2f}, timeout rate: : {:.2f}, avg nav time: {:.2f}s, collision frequency: {:.2f}, freezing frequency: {:.2f}, total reward: {:.4f}'.
                     format(phase.upper(), extra_info, success_rate, timeout_rate, avg_nav_time, collision_rate, frozen_rate,
                            average(cumulative_rewards)))
        if phase in ['val', 'test']:
            logging.info('Frequency of being in danger: %.2f and average min separate distance in danger: %.2f',
                         too_close / num_timesteps, np.mean(np.array(min_dist)[:,1]))

        if print_failure:
            logging.info('Timeout cases (case IDs): ' + ' '.join([str(x) for x in timeout_cases]))
            logging.info('Collision cases (case IDs): ' + ' '.join([str(x) for x in collision_cases]))
