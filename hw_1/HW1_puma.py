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
    puma = env.ReadRobotXMLFile('robots/puma.robot.xml')
    env.Add(puma)

    # PR2 is at -3.4, -1.4, 0.05

    # put puma infront of PR2
    puma.SetTransform(array([[1, 0, 0, -2.65],
                              [0, 1, 0, -1.4],
                              [0, 0, 1, 0.0],
                              [0, 0, 0, 1]]))

    # check for contact
    if env.CheckCollision(robot, puma):
        print('Robots are colliding when they should not be')

    # slap the puma with PR2's left arm
    with env:
        jointnames = ['l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        #robot.SetActiveDOFValues([1.29023451,-2.32099996,-0.69800004]);
        robot.SetActiveDOFValues([1.29023451,-1.75,0.99]);
        robot.GetController().SetDesired(robot.GetDOFValues());
    waitrobot(robot)

    # check for successful contact
    if env.CheckCollision(robot, puma):
        print('Robots are colliding as expected')


    #### END OF YOUR CODE ###


    raw_input("Press enter to exit...")

