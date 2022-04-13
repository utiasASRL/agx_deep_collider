import numpy as np
from crowd_sim.envs.policy.policy import Policy
from crowd_sim.envs.utils.action import ActionRot
import simple_nav.utils.PythonRobotics.dynamic_window_approach as PRdwa
from simple_nav.utils.PythonRobotics.dynamic_window_approach import RobotType

class DynamicWindowApproach(Policy):
    """
    Template to implement a teleoperation.
    This placeholder is a copy of crowd_sim.envs.policy.linear
    """

    def __init__(self):
        super().__init__()
        self.name = "DWA"
        self.trainable = False
        self.kinematics = 'unicycle'
        self.multiagent_training = True
        # self.safety_space = 0
        # self.neighbor_dist = 10
        # self.max_neighbors = 10
        self.time_horizon = 2.5
        self.radius = 0.3
        self.max_speed = 0.8
        self.sim_config = None

    def configure(self, config):
        pass

    def configure_dwa(self, config, section):
        self.sim_config = PRdwa.Config()
        self.radius = config.getfloat(section, 'radius')
        self.max_speed = config.getfloat(section, 'v_pref')
        self.sim_config.max_speed = self.max_speed
        self.sim_config.min_speed = -self.max_speed
        self.sim_config.max_accel = config.getfloat(section, 'acc_max')  # [m/ss]
        self.sim_config.max_delta_yaw_rate = config.getfloat(section, 'ang_acc_max')  # [rad/ss]
        self.sim_config.v_resolution = 0.05  # [m/s]
        self.sim_config.yaw_rate_resolution = 1 * np.pi / 180.0  # [rad/s]
        self.sim_config.dt = self.time_step  # [s] Time tick for motion prediction
        self.sim_config.predict_time = self.time_horizon  # [s]
        self.sim_config.to_goal_cost_gain = 2
        self.sim_config.speed_cost_gain = 2.0
        self.sim_config.obstacle_cost_gain = 15.5
        self.sim_config.robot_stuck_flag_cons = 0.01  # constant to prevent robot stucked
        self.sim_config.robot_type = RobotType.circle
        self.sim_config.robot_radius = self.radius  # [m] for collision check
        # self.sim_config.robot_width = 0.5  # [m] for collision check
        # self.sim_config.robot_length = 1.2  # [m] for collision check
        self.sim_config.ob = np.zeros((1, 2))

    def predict(self, state):
        """
        Create a PythonRobotics DWA simulation at each time step and run one step
        PythonRobotics: https://github.com/AtsushiSakai/PythonRobotics
        PythonRobotics Path Planning docs: https://pythonrobotics.readthedocs.io/en/latest/modules/path_planning/path_planning.html

        # Function structure based on CrowdSim ORCA

        :param state:
        :return:
        """
        self_state = state.self_state
        # make a sim config if it is the first time step
        if self.sim_config is None:
            self.sim_config = PRdwa.Config()
            self.sim_config.max_speed = self.max_speed
            self.sim_config.min_speed = -self.max_speed
            # self.sim_config.max_yaw_rate = 40.0 * np.pi / 180.0  # [rad/s]
            self.sim_config.max_accel = 0.5  # [m/ss]
            self.sim_config.max_delta_yaw_rate = 45.0 * np.pi / 180.0  # [rad/ss]
            self.sim_config.v_resolution = 0.05  # [m/s]
            self.sim_config.yaw_rate_resolution = 1 * np.pi / 180.0  # [rad/s]
            self.sim_config.dt = self.time_step  # [s] Time tick for motion prediction
            self.sim_config.predict_time = self.time_horizon  # [s]
            self.sim_config.to_goal_cost_gain = 2
            self.sim_config.speed_cost_gain = 2.0
            self.sim_config.obstacle_cost_gain = 15.5
            self.sim_config.robot_stuck_flag_cons = 0.01  # constant to prevent robot stucked
            self.sim_config.robot_type = RobotType.circle
            self.sim_config.robot_radius = self.radius  # [m] for collision check
            # self.sim_config.robot_width = 0.5  # [m] for collision check
            # self.sim_config.robot_length = 1.2  # [m] for collision check
            self.sim_config.ob = np.zeros((len(state.human_states), 2))

        # if there are a different number of obstacles
        if len(self.sim_config.ob) != len(state.human_states):
            self.sim_config.ob = np.zeros((len(state.human_states), 2))

        # update state of obstacles for the sim_config
        for idx, human_state in enumerate(state.human_states):
            self.sim_config.ob[idx, :] = human_state.position
        PRdwa_state = [self_state.px, self_state.py, self_state.theta, self_state.vx*np.cos(self_state.theta)+self_state.vy*np.sin(self_state.theta), 0]

        u, _ = PRdwa.dwa_control(PRdwa_state, self.sim_config, self_state.goal_position, self.sim_config.ob)
        action = ActionRot(u[0], u[1])
        # run dwa formula


        # Linear code
        # self_state = state.self_state
        # theta = np.arctan2(self_state.gy-self_state.py, self_state.gx-self_state.px)
        # vx = np.cos(theta) * self_state.v_pref
        # vy = np.sin(theta) * self_state.v_pref
        # action = ActionXY(vx, vy)

        return action
