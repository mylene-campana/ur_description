#/usr/bin/env python
# Script which goes with ur_description package.
# Load 6-DoF arm robot to test methods.

from hpp.corbaserver.ur5_robot import Robot
from hpp.corbaserver import Client
from hpp.corbaserver import ProblemSolver

robot = Robot ('ur5')
cl = robot.client
ps = ProblemSolver (robot)

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

#ps.readRoadmap ('/local/mcampana/devel/hpp/data/ur5-RRT.rdm')
#ps.readRoadmap ('/local/mcampana/devel/hpp/data/ur5-RRT1.rdm')

#ps.selectPathPlanner ("VisibilityPrmPlanner")
#ps.selectPathValidation ("Dichotomy", 0.)

ps.solve ()
ps.pathLength(0)

ps.saveRoadmap ('/local/mcampana/devel/hpp/data/ur5-RRT-testConv1.rdm')

from viewer_display_library_OPTIM import plotPointBodyCurvPath
dt = 0.06

jointName = 'wrist_3_joint'
plotPointBodyCurvPath (r, cl, robot, dt, 0, jointName, [0.28,0,0], 'pathPoint_'+jointName, [0.9,0.1,0.1,1])

ps.addPathOptimizer('RandomShortcut')
ps.optimizePath (0)
ps.pathLength(1)

plotPointBodyCurvPath (r, cl, robot, dt, 1, jointName, [0.28,0,0], 'pathPointRS_'+jointName, [0.1,0.1,0.9,1])

#ps.clearPathOptimizers()
#ps.addPathOptimizer('PartialShortcut')
#ps.optimizePath (0)
#ps.pathLength(2)

ps.clearPathOptimizers()
ps.addPathOptimizer("GradientBased")
ps.optimizePath (0)
#ps.numberPaths()
ps.pathLength(ps.numberPaths()-1)


plotPointBodyCurvPath (r, cl, robot, dt, ps.numberPaths ()-1, jointName, [0.28,0,0], 'pathPointGB_'+jointName, [0.2,0.9,0.2,1])


qColl = [-1.09063,-2.10431,0.382719,0.12038,0.535622,-1.95976]
qFree = [-1.09063,-2.10431,0.382716,0.120381,0.535621,-1.95977] # dist 1e-6 to nearest obst...


# when qColl is in collision but without FCL contacts (precision probably not enough in debug)
qColl = [-1.24166,-2.30012,2.57358,-1.95423,1.23529,1.18378]
qFree = [-1.24166,-2.30012,2.57358,-1.95423,1.23529,1.18378]


pp(ps.numberPaths()-1)

r(q2)

len(ps.getWaypoints (0))

# Add light to scene
lightName = "li"
r.client.gui.addLight (lightName, r.windowId, 0.1, [0.8,0.8,0.8,0.7])
r.client.gui.addToGroup (lightName, r.sceneName)
r.client.gui.applyConfiguration (lightName, [-25,0,0,1,0,0,0])
r.client.gui.refresh ()

# Plot or remove frame
from viewer_display_library import plotFrame
plotFrame (r, "framy", [0,0,0], 0.5)
r.client.gui.removeFromGroup ("frame1"+"framy",r.sceneName)
r.client.gui.removeFromGroup ("frame2"+"framy",r.sceneName)
r.client.gui.removeFromGroup ("frame3"+"framy",r.sceneName)

import time
pp.dt = 0.02
r(q1)
r.startCapture ("capture","png")
r(q1)
#pp(0)
pp(ps.numberPaths()-1)
r(q2); time.sleep(1);
r.stopCapture ()

# Load obstacles in HPP #
cl.obstacle.loadObstacleModel('ur_description','obstacles','') # cylinders
cl.obstacle.loadObstacleModel('ur_description','table','')
cl.obstacle.loadObstacleModel('ur_description','wall','') # wall with hole

## ffmpeg commands
ffmpeg -r 50 -i capture_0_%d.png -r 25 -vcodec libx264 video.mp4
x=0; for i in *png; do counter=$(printf %03d $x); ln "$i" new"$counter".png; x=$(($x+1)); done
ffmpeg -r 50 -i new%03d.png -r 25 -vcodec libx264 video.mp4
mencoder video.mp4 -channels 6 -ovc xvid -xvidencopts fixed_quant=4 -vf harddup -oac pcm -o video.avi

## DEBUG commands
cl.robot.setCurrentConfig(q1)
cl.robot.collisionTest()
res=cl.robot.distancesToCollision()
from numpy import *
res[1][argmin(res[0])]
res[2][argmin(res[0])]
r( cl.problem.configAtDistance(0,5) )
cl.problem.optimizePath (0)
cl.problem.clearRoadmap ()
cl.problem.resetGoalConfigs ()
robot.getJointNames ()
robot.getConfigSize ()
cl.obstacle.getObstaclePosition('decor_base')
robot.getJointOuterObjects('shoulder_pan_joint')
r([-0.4, -2, -1.8, 0, 0.8, 0])
robot.isConfigValid([-0.4, -2, -1.8, 0, 0.8, 0]) # collision not detected with complete mesh


## Debug Optimization Tools ##############
num_log = 32174
from parseLog import parseCollConstrPoints, parseNodes

collConstrNodes = parseNodes (num_log, '189: qFree_ = ')
collNodes = parseNodes (num_log, '182: qColl = ')

contactPoints = parseCollConstrPoints (num_log, '77: contact point = (')
x1_J1 = parseCollConstrPoints (num_log, '96: x1 in R0 = (')
x2_J1 = parseCollConstrPoints (num_log, '97: x2 in R0 = (')
x1_J2 = parseCollConstrPoints (num_log, '116: x1 in J2 = (')
x2_J2 = parseCollConstrPoints (num_log, '117: x2 in J2 = (') #x2_J2 <=> contactPoints


## same with viewer !
from viewer_display_library_OPTIM import transformInConfig, plotPoints, plotPointsAndLines, plot2DBaseCurvPath, plotDofCurvPath, plotPointBodyCurvPath, plotBodyCurvPath
contactPointsViewer = transformInConfig (contactPoints)
x1_J1Viewer = transformInConfig (x1_J1)
x2_J1Viewer = transformInConfig (x2_J1)
x1_J2Viewer = transformInConfig (x1_J2)
x2_J2Viewer = transformInConfig (x2_J2)

# Plot points
sphereNamePrefix = "sphereContactPoints_"
plotPoints (r, sphereNamePrefix, contactPointsViewer, 0.02)
sphereSize=0.01
lineNamePrefix = "lineJ1_"; sphereNamePrefix = "sphereJ1_"
plotPointsAndLines (r, lineNamePrefix, sphereNamePrefix, x1_J1Viewer, x2_J1Viewer, sphereSize)
lineNamePrefix = "lineJ2_"; sphereNamePrefix = "sphereJ2_"
plotPointsAndLines (r, lineNamePrefix, sphereNamePrefix, x1_J2Viewer, x2_J2Viewer, sphereSize)

# Plot trajectories
from viewer_display_library_OPTIM import plotPointBodyCurvPath
dt = 0.06

jointName = 'wrist_3_joint'
plotPointBodyCurvPath (r, cl, robot, dt, 0, jointName, [0.28,0,0], 'pathPoint_'+jointName, [0.9,0.1,0.1,1])
plotPointBodyCurvPath (r, cl, robot, dt, 1, jointName, [0.28,0,0], 'pathPointRS_'+jointName, [0.1,0.1,0.9,1])
plotPointBodyCurvPath (r, cl, robot, dt, ps.numberPaths ()-1, jointName, [0.28,0,0], 'pathPointGB_'+jointName, [0.2,0.9,0.2,1])



# test function in cl.robot
jointPosition = robot.getJointPosition ('wrist_3_joint')
pointInJoint = [0.28,0,0]
posAtester = cl.robot.computeGlobalPosition (jointPosition, pointInJoint)

r(q1)
robot.setCurrentConfig (q2)
sphereName = "machin"
r.client.gui.addSphere (sphereName,0.03,[0.1,0.1,0.1,1]) # black
configSphere = posAtester [::]
configSphere.extend ([1,0,0,0])
r.client.gui.applyConfiguration (sphereName,configSphere)
r.client.gui.addToGroup (sphereName, r.sceneName)
r.client.gui.refresh ()

