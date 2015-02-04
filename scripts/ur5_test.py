#/usr/bin/env python
# Script which goes with ur_description package.
# Load 6-DoF arm robot to test methods.
# "roslaunch ur_description ur5.launch"


from hpp_ros import ScenePublisher
from hpp_ros import PathPlayer
from hpp.corbaserver.ur5_robot import Robot
from hpp.corbaserver import Client
import time
import sys
sys.path.append('/local/mcampana/devel/hpp/src/test-hpp/script')

robot = Robot ('ur5')
cl = robot.client
r = ScenePublisher (robot)
#robot.setRootJointPosition([0,0,1.2,1,0,0,0]) # only in HPP, not displayed

# q = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'] 6 DoF#
q1 = [0, -1.57, 1.57, 0, 0, 0]; q2 = [0.2, -1.57, -1.8, 0, 0.8, 0]

cl.problem.setInitialConfig (q1)
cl.problem.addGoalConfig (q2)
p = PathPlayer (cl, r)

# Load obstacles in HPP #
cl.obstacle.loadObstacleModel('ur_description','obstacles','') # cylinders
cl.obstacle.loadObstacleModel('ur_description','table','')
cl.obstacle.loadObstacleModel('ur_description','wall','') # wall with hole

begin=time.time()
cl.problem.solve ()
end=time.time()
print "Solving time: "+str(end-begin)

len(cl.problem.nodes ())
cl.problem.pathLength(0)


# Display obstacles in RViz #
r.addObject('cylinder1','obstacle_base'); r.addObject('cylinder2','l_object_one')
r.addObject('table','table_base'); r.addObject('table1','l_foot_one');
r.addObject('wall1','decor_base'); r.addObject('wall2','l_decor_one');
r.addObject('wall3','l_decor_two'); r.addObject('wall4','l_decor_three');
r(q1)


# Nodes from the roadmap
import time
nodes = cl.problem.nodes ()
len(nodes)
for n in  nodes:
    r (n)
    time.sleep (.7)


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
