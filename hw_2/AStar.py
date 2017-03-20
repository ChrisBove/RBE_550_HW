#!/usr/bin/env python
# -*- coding: utf-8 -*-

# from numpy import dot, zeros
import numpy
from math import sqrt, pi, sin
import copy
from heapq import heapify
try:
	import Queue as Q  # ver. < 3.0
except ImportError:
	import queue as Q

# this is where to switch the heuristic type
def heuristic(configuration1, configuration2):
	return euclideanDistance(configuration1, configuration2) + 0.1*angularDistance(configuration1, configuration2)

# this is where to switch the connectedness
def getNeighbors(node):
	return eightNeighbors(node)

def angularDistance(configuration1, configuration2):
	return abs(configuration1[2] - configuration2[2])

def euclideanDistance(configuration1, configuration2):
	return sqrt(pow(configuration1[0]-configuration2[0],2)+pow(configuration1[1]-configuration2[1],2))

def manhattanDistance(configuration1, configuration2):
	return abs(configuration1[0]-configuration2[0]) + abs(configuration1[1]-configuration2[1])

def isColliding(configuration):
	rob.SetActiveDOFValues(configuration)

	return environ.CheckCollision(rob)

class AStarNode:
	def __init__(self, parent, configuration, g, h): 
		self.parent = parent
		self.configuration = configuration
		self.g = g
		self.h = h
		self.f = g + h
	def setParent(self, parent): 
		self.parent = (parent)
	def __cmp__(self, other):
		return cmp(self.f, other.f)
	def isSameConfig(self, configuration):
		return ((abs(self.configuration[0] - configuration[0])) < resolution) and (abs(self.configuration[1] - configuration[1]) < resolution) and (abs(sin(self.configuration[2]) - sin(configuration[2])) < 0.1)

def eightNeighbors(node):
	configurations = list()

	# generate new configurations in four directions
	configurations.append([node.configuration[0]+resolution, node.configuration[1], 0])
	configurations.append([node.configuration[0], node.configuration[1]+resolution, 0])
	configurations.append([node.configuration[0]-resolution, node.configuration[1], 0])
	configurations.append([node.configuration[0], node.configuration[1]-resolution, 0])
	# diagonals too
	configurations.append([node.configuration[0]+resolution, node.configuration[1]+resolution, 0])
	configurations.append([node.configuration[0]-resolution, node.configuration[1]+resolution, 0])
	configurations.append([node.configuration[0]-resolution, node.configuration[1]-resolution, 0])
	configurations.append([node.configuration[0]+resolution, node.configuration[1]-resolution, 0])
	
	return generateChildNodesFromConfigurations(node, configurations)

def fourNeighbors(node):
	configurations = list()

	# generate new configurations in four directions
	configurations.append([node.configuration[0]+resolution, node.configuration[1], 0])
	configurations.append([node.configuration[0], node.configuration[1]+resolution, 0])
	configurations.append([node.configuration[0]-resolution, node.configuration[1], 0])
	configurations.append([node.configuration[0], node.configuration[1]-resolution, 0])
	
	return generateChildNodesFromConfigurations(node, configurations)

def generateChildNodesFromConfigurations(node, configurations):
	neighbors = list()
	# first add the remaining angles
	length_of_configurations = len(configurations)
	step_size = pi/2
	for i in range(0,length_of_configurations):
		# didn't work so well - I think openrave only does -pi to pi instead of 0 to 2pi
		# for angle in range (1, 4):
		# 	configurations.append([configurations[i][0], configurations[i][1], angle*step_size])
		configurations.append([configurations[i][0], configurations[i][1], step_size])
		configurations.append([configurations[i][0], configurations[i][1], 2*step_size])
		configurations.append([configurations[i][0], configurations[i][1], -step_size])

	for configuration in configurations:
		# check if configuration is valid (no collision)
		if configuration in collisionSet:
			continue
		elif not isColliding(configuration):
			# if not a collision, create node and add to neighbors
			neighbors.append(AStarNode(node, configuration, node.g + euclideanDistance(node.configuration, configuration) + 0.1*angularDistance(node.configuration, configuration), heuristic(configuration, goalConfig)))
		else:
			collisionSet.append(configuration)

	return neighbors

def configIsInSet(config, set):
	for c in set:
		if numpy.array_equal(c, config):
			return True
	return False

def isNodeInSet(node, set):
	for config in set:
		if node.isSameConfig(config.configuration):
			return True
	return False

def isInSetAndUpdate(node, set, resort):
	for config in set:
		if node.isSameConfig(config.configuration):
			# if node has lower f than same one in the set
			# TODO CHECK THIS OUT
			opennedConfig = config
			if node.g < opennedConfig.g:
				opennedConfig.g = node.g
				opennedConfig.f = node.f
				opennedConfig.parent = node.parent
				if resort:
					heapify(set) # reprioritize
			return True
			# if this is in the set, but this one has a lower f score, we need to remove the other and add this
	return False

def generatePath(startNode, endNode):
	global finalPath
	path = list()
	path.append(endNode.configuration)

	iterator = endNode
	while not iterator.isSameConfig(startNode.configuration):
		iterator = iterator.parent
		path.append(iterator.configuration)

	path.append(startNode.configuration)

	path.reverse()

	finalPath = copy.copy(path)

	return finalPath


def aStar(startConfig, endConfig, robot, env):
	global path, openSet, closedSet, resolution, rob, environ, goalConfig, collisionSet
	resolution = 0.05
	rob = robot
	environ = env
	goalConfig = endConfig

	path = list()
	openSet = Q.PriorityQueue()
	closedSet = list()
	collisionSet = list()

	startNode = AStarNode(None, startConfig, 0, heuristic(startConfig, endConfig))

	# check if start is the end goal
	if startNode.isSameConfig(endConfig):
		path.append(startNode.configuration)
		print('The goal is the same as the starting point!')
		return path

	# check if robot is already in collision state
	if isColliding(startConfig):
		print('ERROR: The robot is starting in a collision state.')
		return None

	# put start in the open set
	openSet.put(startNode)

	print('Starting search...')

	try:
		# while open list not empty
		while not openSet.empty():
			# print(openSet.qsize())
			# pop node Q off the list (the cheapest f cost one)
			q = openSet.get()
			# generate the neighbors, setting parent to Q
			neighbors = getNeighbors(q)
			# for each neighbor
			for neighbor in neighbors:
				# if neighbor is the goal, stop the search - save the path
				if neighbor.isSameConfig(endConfig):
					print('GOAL has been found!!!')
					rob.SetActiveDOFValues(startConfig) # so robot hasn't moved
					return generatePath(startNode, neighbor)

				# if neighbor is in closedSet but with lower f, skip it
				if isInSetAndUpdate(neighbor, closedSet, False):
					continue
				# if neighbor is in openSet but with lower f, skip it
				if isInSetAndUpdate(neighbor, openSet.queue, True):
					continue
				# else, add neighbor to open list
				openSet.put(neighbor)
			# put Q on the closed list
			closedSet.append(q)
	except KeyboardInterrupt:
		print('Keyboard interrupted search.')
		rob.SetActiveDOFValues(startConfig) # so robot hasn't moved
		pass

	print('No Solution Found.')
	rob.SetActiveDOFValues(startConfig) # so robot hasn't moved
	return path

def getDrawablePath():
	path_positions = numpy.zeros((len(finalPath),3))
	for i in range (0,len(finalPath)):
		path_positions[i,0] = finalPath[i][0]
		path_positions[i,1] = finalPath[i][1]
		path_positions[i,2] = 0.15
	return copy.copy(path_positions)
	
def getDrawableSearchedSpace():
	space_positions = numpy.zeros((len(closedSet),3))
	for i in range (0,len(closedSet)):
		space_positions[i,0] = closedSet[i].configuration[0]
		space_positions[i,1] = closedSet[i].configuration[1]
		space_positions[i,2] = 0.11
	return copy.copy(space_positions)

def getDrawableCollisionSpace():
	positions = numpy.zeros((len(collisionSet),3))
	for i in range (0,len(collisionSet)):
		positions[i,0] = collisionSet[i][0]
		positions[i,1] = collisionSet[i][1]
		positions[i,2] = 0.13
	return copy.copy(positions)