from deepbots.supervisor.controllers.robot_supervisor import RobotSupervisor
from utilities import normalizeToRange, plotData
from PPO_agent import PPOAgent, Transition

from gym.spaces import Box, Discrete
import numpy as np


class CartpoleRobot(RobotSupervisor):
    def __init__(self):
        super().__init__()

        self.observation_space = Box(low=np.array([-0.4, -np.inf, -1.3, -np.inf]),
                                    high=np.array([0.4,np.inf,1.3,np.inf]),
                                    dtype=np.float64)
        self.action_space = Discrete(2)

        self.robot = self.getSelf()
        self.positionSensor = self.getDevice("polePosSensor")
        self.positionSensor.enable(self.timestep)

        self.poleEndpoint = self.getFromDef("POLE_ENDPOINT")
        self.wheels = []
        for wheelName in ['wheel1','wheel2','wheel3','wheel4']:
            wheel = self.getDevice(wheelName)
            wheel.setPosition(float('inf'))
            wheel.setVelocity(0.0)
            self.wheels.append(wheel)

        self.stepsPerEpisode = 200
        self.episodeScore = 0
        self.episodeScoreList = []

    def get_observations(self):
        # Position: on z axis
        cartPosition = normalizeToRange(self.robot.getPosition()[2], -0.4, 0.4, -1.0, 1.0)
        # Linear velocity: on z axis
        cartVelocity = normalizeToRange(self.robot.getVelocity()[2], -0.2, 0.2, -1.0, 1.0, clip=True)
        # Pole angle: off vertical
        poleAngle = normalizeToRange(self.positionSensor.getvalue(), -0.23, 0.23, -1.0, 1.0, clip=True)
        # Angular velocity x of poleEndpoint
        endpointVelocity = normalizeToRange(self.poleEndpoint.getVelocity()[3], -1.5, 1.5, -1.0, 1.0, clip=True)

        return [cartPosition, cartVelocity, poleAngle, endpointVelocity]

    def get_reward(self, action=None):
        # Simple reward: agent gets +1 reward for each step it manages to keep the pole from falling
        return 1

    def is_done(self):
        # 195/200 steps good
        if self.episodeScore > 195.0:
            return True

        poleAngle = round(self.positionSensor.getValue(), 2)
        if abs(poleAngle) > .261799388 # 15 degrees off vertical
            return True

        cartPosition = round(self.robot.getPosition()[2], 2) # Position: z-axis
        if abs(cartPosition) > 0.39:
            return True

        return False

    def solved(self):
        if len(self.episodeScoreList) > 100: # Over 100 trials thus far
            if np.mean(self.episodeScoreList[-100:]) > 195.0:
                return True
        return False

    def get_default_observation(self):
        return [0.0 for _ in range(self.observation_space.shape[0])]

    def apply_action(self,action):
        action = int(action[0])

        if action == 0:
            motorSpeed = 5.0
        else:
            motorSpeed = -5.0

        for i in range(len(self.wheels)):
            self.wheels[i].setPosition(float('inf'))
            self.wheels[i].setVelocity(motorSpeed)

    # Dummy version of render()
    def render(self,mode='human'):
        print("render() is not used")

    # Dummy version of get_info()
    def get_info(self):
        return None
    
