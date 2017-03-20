#!/usr/bin/env python
# -*- coding: utf-8 -*-
#HW1 for RBE 595/CS 525 Motion Planning
#code based on the simplemanipulation.py example
import time
import openravepy

#### YOUR IMPORTS GO HERE ####
import AStar

#### END OF YOUR IMPORTS ####

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)

def tuckarms(env,robot):
    with env:
        jointnames = ['l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint','r_shoulder_lift_joint','r_elbow_flex_joint','r_wrist_flex_joint']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        robot.SetActiveDOFValues([1.29023451,-2.32099996,-0.69800004,1.27843491,-2.32100002,-0.69799996]);
        robot.GetController().SetDesired(robot.GetDOFValues());
    waitrobot(robot)

if __name__ == "__main__":

    env = Environment()
    env.SetViewer('qtcoin')
    env.Reset()
    # load a scene from ProjectRoom environment XML file
    env.Load('data/pr2test2.env.xml')
    time.sleep(0.1)

    # 1) get the 1st robot that is inside the loaded scene
    # 2) assign it to the variable named 'robot'
    robot = env.GetRobots()[0]

    # tuck in the PR2's arms for driving
    tuckarms(env,robot);


    with env:
        # the active DOF are translation in X and Y and rotation about the Z axis of the base of the robot.
        robot.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.RotationAxis,[0,0,1])

        goalconfig = [2.6,-1.3,-pi/2]
        #### YOUR CODE HERE ####
        # goalconfig = [-3,-1.3,-pi/2] # for quick testing
        dofValues = robot.GetActiveDOFValues()
        startconfig = [dofValues[0], dofValues[1], dofValues[2]]

        #### Implement the A* algorithm to compute a path for the robot's base starting from the current configuration of the robot and ending at goalconfig. The robot's base DOF have already been set as active. It may be easier to implement this as a function in a separate file and call it here.
        startTime = time.time()
        path = AStar.aStar(startconfig, goalconfig, robot, env)

        doneTime = time.time()
        elapsedTime = doneTime - startTime
        print 'Elapsed time: ', elapsedTime

        #### Draw your path in the openrave here (see /usr/lib/python2.7/dist-packages/openravepy/_openravepy_0_8/examples/tutorial_plotting.py for examples)
        try:
            handles = []
            handles.append(env.plot3(points=AStar.getDrawablePath(),
                                   pointsize=15,
                                   colors=array((0,0,0))))
            # draw in black

            #### Draw the X and Y components of the configurations explored by A*
            # draw collision-free explored space in blue
            handles.append(env.plot3(points=AStar.getDrawableSearchedSpace(),
                                   pointsize=15,
                                   colors=array((0,0,1))))
            # collision space in red
            handles.append(env.plot3(points=AStar.getDrawableCollisionSpace(),
                                   pointsize=15,
                                   colors=array((1,0,0))))
        # Hack for if we don't have any collisions along our path or other position arrays are empty for drawing
        except :
            pass

        #### Now that you have computed a path, execute it on the robot using the controller. You will need to convert it into an openrave trajectory. You can set any reasonable timing for the configurations in the path. Then, execute the trajectory using robot.GetController().SetPath(mypath);
        traj = openravepy.RaveCreateTrajectory(env, '')
        cSpaceSpec = robot.GetActiveConfigurationSpecification('linear') # want linear interpolation

        count = 0
        for config in path:
            if count == 0:
                traj.Init(cSpaceSpec)
            traj.Insert(count, config)
            count = count + 1

        openravepy.planningutils.RetimeActiveDOFTrajectory(traj, robot, False, 0.5, 0.5, "LinearTrajectoryRetimer", "")

        robot.GetController().SetPath(traj)
        print('Trajectory sent to robot. Waiting for robot to move.')

        #### END OF YOUR CODE ###
    waitrobot(robot)

    raw_input("Press enter to exit...")

