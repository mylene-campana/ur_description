#/usr/bin/env python
# Script which goes with ur_description package.
# Load 6-DoF arm robot to test methods.

from hpp.corbaserver.ur5_robot import Robot
from hpp.corbaserver import Client
from hpp.corbaserver import ProblemSolver

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



#ps.saveRoadmap ('/local/mcampana/devel/hpp/data/ur5-RRT.rdm') # not returning the same sol..
#ps.selectPathPlanner ("VisibilityPrmPlanner")
#ps.saveRoadmap ('/local/mcampana/devel/hpp/data/ur5-PRM.rdm')
ps.readRoadmap ('/local/mcampana/devel/hpp/data/ur5-RRT.rdm')

ps.solve ()
ps.pathLength(0)

ps.addPathOptimizer("GradientBased")
ps.optimizePath (0)
ps.numberPaths()
ps.pathLength(ps.numberPaths()-1)
pp(ps.numberPaths()-1)

ps.clearPathOptimizers()
ps.addPathOptimizer('RandomShortcut')
ps.optimizePath (0)
ps.pathLength(1)

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

pp.dt = 0.02
r.startCapture ("capture","png")
#pp(1)
pp(ps.numberPaths()-1)
r.stopCapture ()

# Load obstacles in HPP #
cl.obstacle.loadObstacleModel('ur_description','obstacles','') # cylinders
cl.obstacle.loadObstacleModel('ur_description','table','')
cl.obstacle.loadObstacleModel('ur_description','wall','') # wall with hole

## ffmpeg commands
ffmpeg -r 50 -i capture_0_%d.png -r 25 -vcodec libx264 video.mp4
x=0; for i in *png; do counter=$(printf %03d $x); ln "$i" new"$counter".png; x=$(($x+1)); done
ffmpeg -r 50 -i new%03d.png -r 25 -vcodec libx264 video.mp4


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
