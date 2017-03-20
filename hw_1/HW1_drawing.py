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
    handles = []

    # draw red rectangles around every table's boundaries
    length = 1.2
    width = 0.6
    half_l = length/2.0
    half_w = width/2.0
    # table ID's are 2-7, for all tables, draw rectangles around bounds
    for id_number in range (2, 8):
        table = env.GetBodyFromEnvironmentId(id_number)
        transform = table.GetTransform()
        
        position = [transform[0,3],transform[1,3],transform[2,3]]
        cos_t = transform[1,0]
        sin_t = transform[0,0]

        #calculate each rectangle corner position from center position and angle of table
        lt_corner = [transform[0,3]-cos_t*half_l -sin_t*half_w, transform[1,3]+cos_t*half_w +sin_t*half_l, transform[2,3]]
        lb_corner = [transform[0,3]-cos_t*half_l -sin_t*half_w, transform[1,3]-cos_t*half_w -sin_t*half_l, transform[2,3]]
        rt_corner = [transform[0,3]+cos_t*half_l +sin_t*half_w, transform[1,3]+cos_t*half_w +sin_t*half_l, transform[2,3]]
        rb_corner = [transform[0,3]+cos_t*half_l +sin_t*half_w, transform[1,3]-cos_t*half_w -sin_t*half_l, transform[2,3]]

        # draw lines from each point and wrap back around
        handles.append(env.drawlinestrip(points=array((lt_corner,lb_corner,rb_corner,rt_corner,lt_corner)),
                                           linewidth=3.0,
                                           colors=array((1,0,0))))


    # draw 35 blue points in a circle around environment
    # world ID is 1
    world = env.GetBodyFromEnvironmentId(1)
    # we know env is at origin, so no need to offset the circle

    # initialize array of circle points spaced evenly around perimeter
    radius = 4.5
    positions = numpy.zeros((36, 3))
    for i in range (0, 36):
        positions[i, 0] = radius*math.cos(math.radians(i*10))
        positions[i, 1] = radius*math.sin(math.radians(i*10))


    # plot all the points as a cloud at once
    handles.append(env.plot3(points=positions,
                                   pointsize=0.1,
                                   colors=array((0,0,1)),
                                   drawstyle=1))

    #### END OF YOUR CODE ###


    raw_input("Press enter to exit...")

    handles = None