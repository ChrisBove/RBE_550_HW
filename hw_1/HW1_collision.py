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

    #put PR2 facing wall but not touching
    # put puma infront of PR2
    robot.SetTransform(array([[1, 0, 0, 0],
                              [0, 1, 0, 0],
                              [0, 0, 1, 0.05],
                              [0, 0, 0, 1]]))

    # print out result of call to Check Collision
    if env.CheckCollision(robot):
        print('Robot is colliding when it should not be')
    else:
        print('Check collision is false as expected')

    # set 7 joints of right arm to active
    jointnames = ['r_shoulder_pan_joint','r_shoulder_lift_joint','r_upper_arm_roll_joint','r_elbow_flex_joint',
                    'r_forearm_roll_joint','r_wrist_flex_joint','r_wrist_roll_joint']
    robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])

    # inside with env: block, SetActiveDOFValues to set configuration so it is point straight forward (no changes in viewer)
    with env:
        robot.SetActiveDOFValues([0, 0, 0, -0.1501, 0, -0.101, 0])

        # call check collision to make sure it is colliding with wall and print
        if env.CheckCollision(robot):
            print('Robot is colliding as it should be')
        else:
            print('Robot was not colliding when it should be')
        # use SetDesired() on the robot controller to go to new configuration
        robot.GetController().SetDesired(robot.GetDOFValues())

    # use waitRobot to ensure it reaches before program termination
    waitrobot(robot)

    #### END OF YOUR CODE ###


    raw_input("Press enter to exit...")

