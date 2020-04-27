from OpenGL.GL import *
from .geometric import *
from ..spaces.objectives import *
from ..spaces.statespace import *
from ..spaces.configurationspace import *
from ..spaces.edgechecker import *
from ..spaces.metric import *
from ..spaces.biassets import BoxBiasSet
from ..planners.problem import PlanningProblem

class DoubleIntegratorVisualizer:
    def __init__(self,workspace):
        self.base = workspace

    def toScreen(self,q):
        return q[0],q[1]

    def toState(self,x,y):
        return (x,y,0,0)

    def drawObstaclesGL(self):
        self.base.drawObstaclesGL()

    def drawVerticesGL(self,qs):
        self.base.drawVerticesGL(qs)

    def drawRobotGL(self,q):
        glColor3f(0,0,1)
        glPointSize(7.0)
        self.drawVerticesGL([q])
        l = 0.05
        glBegin(GL_LINES)
        glVertex2f(q[0],q[1])
        glVertex2f(q[0]+l*q[2],q[1]+l*q[3])
        glEnd()

    def drawGoalGL(self,goal):
        self.base.drawGoalGL(goal)

    def drawInterpolatorGL(self,interpolator):
        self.base.drawInterpolatorGL(interpolator)


def doubleIntegratorTest(start_coord=[0.06, 0.25], goal_coord=[0.94, 0.25]):
    cspace = Geometric2DCSpace()
    # add obstacles here:
    #cspace.addObstacle(Circle(0.5,0.5,0.1))
    cspace.addObstacle(Box(0.4166, 0.98333, 0.42, 0.5666))
    cspace.addObstacle(Box(0.4166, 0.43333, 0.42, 0.0166))
    cspace.addObstacle(Box(0.5833, 0.98333, 0.59, 0.7))
    cspace.addObstacle(Box(0.5833, 0.61666, 0.59, 0.3833))
    cspace.addObstacle(Box(0.5833, 0.3, 0.59, 0.0166))


    vspace = BoxConfigurationSpace([-1,-1],[1,1])
    aspace = BoxSet([-5,-5],[5,5])
    aspace.box = BoxBiasSet(aspace.bmin,aspace.bmax,10)
    #start = [0.06,0.25,0,0]
    #goal = [0.94,0.25,0,0]
    start = [start_coord[0], start_coord[1], 0, 0]
    goal = [goal_coord[0], goal_coord[1], 0, 0]
    objective = TimeObjectiveFunction()
    goalRadius = 0.2
    controlSpace = CVControlSpace(cspace,vspace,aspace,dt=0.05,dtmax=0.5)
    '''return PlanningProblem(controlSpace,start,goal,
                           objective=objective,
                           visualizer=DoubleIntegratorVisualizer(cspace),
                           goalRadius = goalRadius,
                           euclidean = True)'''

    plan_prob = PlanningProblem(controlSpace,start,goal,
                          objective=objective,
                          visualizer=DoubleIntegratorVisualizer(cspace),
                          goalRadius = goalRadius,
                          euclidean = True)
    #import pdb; pdb.set_trace()
    return plan_prob
