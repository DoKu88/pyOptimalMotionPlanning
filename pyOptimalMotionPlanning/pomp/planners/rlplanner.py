import gym
# This is essentially just like the CostSpaceRRT from kinodynamics.py but will use RL instead of
# random sampling to sample the next step.

class LearnedCostSpaceSampler(Sampler):
    def __init(self, baseSampler, costMax):
        self.baseSampler = self.baseSampler
        self.costMax = costMax

    def sample(self):
        # TODO: Should use RL model to sample here.
        # TODO: This needs to be trained using heuristic from CostSpace
        pass

# Differences:
# - Call LearnedCostSpaceSampler instead of normal
class RLCostSpaceRRT:
    """ The cost-space Rapidly-exploring Random Tree Planner with RL"""
    def __init__(self, controlSpace, objective, metric, edgeChecker, **params):
        """ Given a ControlSpace control_space, a metric, and an edge checker"""
        self.objective = objective
        self.baseSpace = controlSpace.configurationSpace()


if __name__ == "__main__":
    # Train the RL model and save the weights so that they can be used for sampling later
    pass