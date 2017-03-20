#!/usr/bin/env python
# -*- coding: utf-8 -*-
#HW1 for RBE 595/CS 525 Motion Planning
#code based on the simplemanipulation.py example
import time
import openravepy

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


    #### YOUR CODE HERE ####

    #table id 7 - (-0.21905, 0.01550, 0.73999) quat=(1,0,0,0)
    table7 = env.GetBodyFromEnvironmentId(7)
    table7.SetTransform(array([[1, 0, 0, -0.22],
                              [0, 1, 0, 0.016],
                              [0, 0, 1, 0.74],
                              [0, 0, 0, 1]]))

    #table id 6 - (-3.21020, -0.01409, 0.73998) quat=(1,0,0,0)
    table6 = env.GetBodyFromEnvironmentId(6)
    table6.SetTransform(array([[1, 0, 0, -3.21],
                              [0, 1, 0, -0.016],
                              [0, 0, 1, 0.74],
                              [0, 0, 0, 1]]))

    #### END OF YOUR CODE ###


    raw_input("Press enter to exit...")

