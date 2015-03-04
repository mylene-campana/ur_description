#/usr/bin/env python
# Script which goes with ur_description package.
# Load 6-DoF arm robot to test methods.

from hpp.corbaserver.ur5_robot import Robot
from hpp.corbaserver import Client
from hpp.corbaserver import ProblemSolver
import time

robot = Robot ('ur5')
cl = robot.client
ps =ProblemSolver (robot)
#robot.setRootJointPosition([0,0,1.2,1,0,0,0]) # only in HPP, not displayed

# q = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'] 6 DoF#
q1 = [0, -1.57, 1.57, 0, 0, 0]; q2 = [0.2, -1.57, -1.8, 0, 0.8, 0]

ps.setInitialConfig (q1); ps.addGoalConfig (q2)

from hpp.gepetto import Viewer, PathPlayer
r = Viewer (ps)
pp = PathPlayer (robot.client, r)
r.loadObstacleModel ("ur_description","obstacles","obstacles")
r.loadObstacleModel ("ur_description","table","table")
r.loadObstacleModel ("ur_description","wall","wall")
r(q1)

# Load obstacles in HPP #
cl.obstacle.loadObstacleModel('ur_description','obstacles','') # cylinders
cl.obstacle.loadObstacleModel('ur_description','table','')
cl.obstacle.loadObstacleModel('ur_description','wall','') # wall with hole

begin=time.time()
ps.solve ()
end=time.time()
print "Solving time: "+str(end-begin)
len(ps.getWaypoints (0))

begin=time.time()
ps.optimizePath (0)
end=time.time()
print "Optim time: "+str(end-begin)
cl.problem.getIterationNumber ()
ps.pathLength(0)
ps.pathLength(1)

begin=time.time()
ps.optimizePath (1)
end=time.time()
print "Optim time: "+str(end-begin)
cl.problem.getIterationNumber ()
ps.pathLength(2)



## DEBUG commands
cl.robot.setCurrentConfig(q1)
cl.robot.collisionTest()
cl.robot.distancesToCollision()
from numpy import *
cl.robot.distancesToCollision()[1][argmin(cl.robot.distancesToCollision()[0])]
cl.robot.distancesToCollision()[2][argmin(cl.robot.distancesToCollision()[0])]
r( cl.problem.configAtDistance(0,5) )
cl.problem.optimizePath (0)
cl.problem.clearRoadmap ()
cl.problem.resetGoalConfigs ()
robot.getJointNames ()
robot.getConfigSize ()
cl.obstacle.getObstaclePosition('decor_base')
robot.getJointOuterObjects('shoulder_pan_joint')
