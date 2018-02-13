#!/usr/bin/env python

# map_interceptor.py
#	Receives the map published from the /openslam_map topic and republishers it to the /map topic. It is possible to alter the map in this node, before replishing it. 

import rospy
import sys

from nav_msgs.msg import MapMetaData
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker

import numpy as np
import copy

#######################################################################
# PARAMETERS 
# TODO: parametrize these as inputs from launch file

# TOPICS [in]
map_topic_in = '/openslam_map'
markers_topic = '/markers'

# TOPICS [out]
map_topic_out = '/map'

# Variables
door_radii = 0.5

#######################################################################

# Receives a map from a topic and publishes the same map to the output topic, allowing making modifications to it
class MapInterceptor:

	# Class Constructor
	def __init__(self, map_topic_in, map_topic_out):
		self.map = OccupancyGrid()
		self.gotMap = False
		self.lockMap = False

		# SUBSCRIBERS
		self.map_sub = rospy.Subscriber(map_topic_in, OccupancyGrid, self.mapCallback)

		# PUBLISHERS
		self.map_pub = rospy.Publisher(map_topic_out, OccupancyGrid, queue_size = 5)

	# Publishes self.map to the topic map_topic_out
	def publishMap(self):
		while self.lockMap == True:
			print "waiting for map to unlock.\n"
		self.map_pub.publish(self.map)

	# Returns map resolution in meters/cell
	def getResolution(self):
		return self.map.info.resolution

	# Receives a 2D position x,y in meters, and returns the corresponding cell position in the map grid
	def getCellPosFromWorldPos(self, world_x, world_y):
		offsetX = world_x - self.map.info.origin.position.x
		offsetY = world_y - self.map.info.origin.position.y
		cell_x = int(round(offsetX/self.map.info.resolution))
		cell_y = int(round(offsetY/self.map.info.resolution))

		return cell_x, cell_y

	# Returns the map data in a numpy array
	def getMapGrid(self):
		while self.lockMap == True:
			print "waiting for map to unlock.\n"

		if not self.gotMap:
			print 'No map received yet.\n'
			return None
		
		return np.asarray(self.map.data).reshape((self.map.info.height, self.map.info.width))

	# Receives a numpy array of map in row-major form. Cannot change the dimensions of original map.
	def setMapGrid(self, grid):

		while self.lockMap == True:
			print "waiting for map to unlock.\n"

		if grid.shape[1] != self.map.info.width or grid.shape[0] != self.map.info.height:
			print '\nError: Cannot change map dimensions.'
			return
		
		self.map.data = list(grid.astype(np.int8).flatten())

	# Callback for map_topic_in topic
	def mapCallback(self, data):
		resl = self.map.info.resolution

		# Using lock variable to prevent threading issues accessing the same memory
		self.lockMap = True

		# If it is the first map
		if self.gotMap == False:
			self.map = data

			# Deep copy data
			self.map.data = list(data.data)
			self.gotMap = True

		# Else, copy the information of the old map to the new map, and update uknown cells.
		else:
			oldWidth = self.map.info.width
			oldHeight = self.map.info.height

			newWidth = data.info.width
			newHeight = data.info.height

			# check if dimensions have changed
			if oldWidth != newWidth or  oldHeight != newHeight:

				# update map dimensions
				newGrid = np.full((newHeight, newWidth), -1,dtype=np.int8)
				oldGrid = np.asarray(self.map.data).reshape((self.map.info.height, self.map.info.width))

				x0 = self.map.info.origin.position.x # Old origin x
				y0 = self.map.info.origin.position.y # Old origin y

				x1 = data.info.origin.position.x # New origin x
				y1 = data.info.origin.position.y # New origin y

				offsetX = int(round(abs(x1 - x0)/resl))
				offsetY = int(round(abs(y1 - y0)/resl))

				# Copy old data to new map..
				newGrid[offsetY:offsetY+oldHeight, offsetX:offsetX+oldWidth] = oldGrid

				# Update map itself
				self.map.info.width = newWidth
				self.map.info.height = newHeight

				self.map.info.origin.position.x = x1
				self.map.info.origin.position.y = y1

				self.map.data = list(newGrid.astype(np.int8).flatten())

			# update cells and map metadata
			nCells = newWidth*newHeight
			for i in xrange(nCells):
				if self.map.data[i] < 0:
					self.map.data[i] = data.data[i]

			self.map.header.stamp = rospy.Time.now()

		self.lockMap = False

		print "map size in cells: w = %d, h = %d" % (self.map.info.width, self.map.info.height)
		print "map size in meters: w = %f, h = %f" % (self.map.info.width*resl, self.map.info.height*resl)
		print "map position (position of the cel [0,0] in the world): trans(%f, %f), rot(%f, %f, %f, %f)" % (self.map.info.origin.position.x, self.map.info.origin.position.y, self.map.info.origin.orientation.x, self.map.info.origin.orientation.y, self.map.info.origin.orientation.z, self.map.info.origin.orientation.w)

# Auxiliary class, used to edit the map based on semantic information, such as position of doors, or laser scans 
class MapDoorEditor:

	# Class Constructor
	def __init__(self, interceptor, markers_topic):

		# List of doors received at markers_topic topic
		self.doors = []

		self.mi = interceptor

		# SUBSCRIBERS
		self.marker_sub = rospy.Subscriber(markers_topic, Marker, self.markerCallback)

	# Draws circles of unknown areas around the doors in the map
	def drawDoorCircles(self, radii):

		# Get radii in cell units
		cell_radii = int(round(1.0*radii/self.mi.getResolution()))

		# Get map
		map_grid = self.mi.getMapGrid()

		if map_grid is None:
			return 
		
		# For each door, draw unknown circle in map grid
		for door in self.doors:

			# Get door position in cells
			x_center_meter = door.pose.position.x
			y_center_meter = door.pose.position.y
			x_center_cell, y_center_cell = self.mi.getCellPosFromWorldPos(x_center_meter, y_center_meter)
			
			# Draw door in the grid
			for x in range(-cell_radii, cell_radii):
				for y in range(-cell_radii, cell_radii):
					if (x**2 + y**2) <=  cell_radii**2:
						xp = x + x_center_cell
						yp = y + y_center_cell

						# Clip value
						xp = max(min(xp, map_grid.shape[1]), 0)
						yp = max(min(yp, map_grid.shape[0]), 0)

						# Set to unknown
						map_grid[yp, xp] = 0
			
			# Set map back
			self.mi.setMapGrid(map_grid)
	
	# Callback for markers_topic topic
	def markerCallback(self, data):

		# If it is a door, store it in doors list if it hasn't been added already.
		if data.ns == 'door':
			added = False
			for door in self.doors:
				# If object has already been added to list
				if door.id == data.id:
					added = True
					break
		
			# Check if it has to be added to list
			if not added:
				# Add door to list
				self.doors.append(data)

def run():
	global map_topic_in
	global map_topic_out
	global markers_topic
	global door_radii

	# Objects
	mi = MapInterceptor(map_topic_in, map_topic_out)
	md = MapDoorEditor(mi, markers_topic)

	# Initialize ros node
	rospy.init_node('map_interceptor', anonymous=True)

	rate = rospy.Rate(3)

	while not rospy.is_shutdown():
		md.drawDoorCircles(door_radii)
		mi.publishMap()
		rate.sleep()

if __name__ == "__main__":
	run()
