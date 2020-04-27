import numpy as np
import gym
from gym.spaces import Box, MultiDiscrete, Tuple
from gym import error, spaces, utils

from ..example_problems.doubleintegrator import DoubleIntegratorVisualizer
from ..spaces.controlspace import vectorops
from ..visualizer import runVisualizer, PlanVisualizationProgram

from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *

# FIXME: These imports should be fixed
from ..planners import allplanners
from ..example_problems import *

# Running this should produce same output as running the main.py function.

# TODO:
# - Compute step space
# - To reset, just go back to initial point

OBSTACLE_REWARD = -1000
DT_MIN = 1  # milliseconds
DT_MAX = 50  # milliseconds
WIDTH = 60
HEIGHT = 60
V_MIN = -25
V_MAX = 25


# TODO

class RLCostSpaceEnv(gym.Env):
    """ Custom gym environment for reinforcement learning tree search
    action_space[3]: dt, u_y, u_x
    observation_space: position, velocity, """
    metadata = {"render.modes": ["human"]}

    def __init__(self, start, goal):
        """ Initializes the environment.
        Args:
            start (float[4])              : [x, y, v_xi, x_yi]
            goal (float[4])               : [x, y, v_xf, v_yf]
            obstacles [height (int) x width (int)]: square matrix with 0 indicating empty and 1 is obstacle
        Returns:
            Initialized object duh"""
        self.start = start
        self.goal = goal
        self.space = DoubleIntegratorVisualizer()
        self.visualizer = PlanVisualizationProgram(self.space.get_planning_problem(self.start, self.goal),
                                                   "ao-rrt", "data/ao-rrt")
        self.visualizer.width = self.visualizer.height = 640
        glutInit([])
        self.visualizer.initWindow()
        self.current_state = start
        self.V = [start]  # Vertices of the graph
        self.E = []  # Edges of the graph
        self.N = 0
        self.new = False
        self.obstacles = None  # TODO
        # Action space is time, u_y, and u_x. Acceleration can be positive or negative.
        self.action_space = Box(np.array([1.0, -1.0, -1.0]), np.array([10.0, 1.0, 1.0]))
        # Observation space is width x height, current coordinates, goal coordinates, v_x, v_y
        self.observation_space = Tuple((MultiDiscrete([60, 60]), MultiDiscrete([60, 60]), MultiDiscrete([60, 60]),
                                        Box(np.array([-25, -25]), np.array([25, 25]))))

    def reward(self):
        # If at obstacle, negative reward
        # TODO
        # Computed as reciprocal Euclidean distance from goal
        [x_i, y_i, v_xi, v_yi] = self.current_state
        [x_f, y_f, v_xf, v_yf] = self.goal
        return 1 / np.sqrt((x_f - x_i) ** 2 + (y_f - y_i) ** 2 + (v_xf - v_xi) ** 2 + (v_yf - v_yi) ** 2)

    def next_state(self, x, u):
        """ Computes and moves to the next state given input and adds it to the graph.
        Args:
            x: [x_i, y_i, v_x, v_y]
            u: [dt, u_x, u_y]

        Returns:
            x: new state computed using acceleration and velocity"""
        n = len(x) // 2
        q, v = x[:n], x[n:]
        dt = u[0]
        a = u[1:]
        assert n == len(a)
        vnew = vectorops.madd(v, a, dt)
        qnew = vectorops.add(vectorops.madd(q, a, 0.5 * dt ** 2), vectorops.mul(v, dt))
        new_state = qnew + vnew
        # Add to the graph
        self.V.append(new_state)
        if self.new:
            self.E.append((0, self.N + 1, u))
            self.new = False
        else:
            self.E.append((self.N, self.N + 1, u))
        self.N += 1
        self.current_state = new_state
        return new_state

    def step(self, action):
        """Run one timestep of the environment's dynamics. When end of
        episode is reached, you are responsible for calling `reset()`
        to reset this environment's state.

        Accepts an action and returns a tuple (observation, reward, done, info).

        Args:
            action (object): an action provided by the agent

        Returns:
            observation (object): agent's observation of the current environment
            reward (float) : amount of reward returned after previous action
            done (bool): whether the episode has ended, in which case further step() calls will return undefined results
            info (dict): contains auxiliary diagnostic information (helpful for debugging, and sometimes learning)
        """
        # IMPORTANT: Divide time by 100 since it is in milliseconds
        action[0] /= 100
        new_state = self.next_state(self.current_state, action)
        return None, self.reward(), None, None

    def sample(self):
        """ Returns a randomly sampled action from the action space. """
        return self.action_space.sample()

    def reset(self):
        """ Resets the state of the environment and returns an initial observation.

        Returns:
            observation (object): the initial observation.
        """
        self.current_state = self.start
        self.new = True
        return None

    def render(self):
        self.visualizer.G = (self.V, self.E)
        self.visualizer.displayfunc()
        glutMainLoopEvent()

    def close(self):
        pass
