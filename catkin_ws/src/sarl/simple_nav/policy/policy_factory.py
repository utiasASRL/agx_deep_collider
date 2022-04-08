from crowd_sim.envs.policy.policy_factory import policy_factory
from crowd_nav.policy.policy_factory import policy_factory
from policy.teleop import Teleop
from policy.dwa import DynamicWindowApproach

policy_factory['teleop'] = Teleop
policy_factory['dwa'] = DynamicWindowApproach
