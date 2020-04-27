from pomp.planners.rl_cost_space_env import RLCostSpaceEnv

env = RLCostSpaceEnv(start=[0.1,0.1, 0.0, 0.0], goal=[0.9,0.9, 0.0, 0.0])
for i in range(25):
    for j in range(50):
        env.render()
        env.step(env.sample())
    env.reset()
