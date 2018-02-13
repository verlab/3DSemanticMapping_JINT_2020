#!/usr/bin/env python

# objet_marker.py :
#	 Subscribes to the /detected_objects_raw topic and publishes filtered detected objects in the map as markers

import rospy
import roslib
import sys
import math
import numpy as np
import struct
import copy
from visualization_msgs.msg import Marker

from object_counter import ObjectCounter

from custom_msgs.msg import WorldObject
# World Object:
#	string objClass
#	float32 x
#	float32 y
#	float64 prob

#######################################################################
# PARAMETERS 
# TODO: parametrize these as inputs from launch file

# TOPICS [in]
objects_topic = '/detected_objects_raw'

# TOPICS [out]
markers_topic = '/markers'

# FRAMES
map_frame = 'map'

# Threshold to print door at location after how many observations of it
marker_thresh = 5

#######################################################################

class ObjectMarker:

	# counter: used to identify each object's id marked in map, uniquely
	max_obj_radii = 1.5

	def __init__(self, objects_topic, markers_topic, map_frame):
		self.map_frame = map_frame
		self.doors = []
		self.counter = 10000

		# SUBSCRIBERS
		self.objects_sub = rospy.Subscriber(objects_topic, WorldObject, self.objectsCallback)

		# PUBLISHERS
		self.marker_pub = rospy.Publisher(markers_topic, Marker, queue_size=10)
		
	def objectsCallback(self, data):
		self.counter+=1

		# Mark object in map as red ball, regardless
		marker = ObjectMarker.getMarker(data.x, data.y, data.objClass+"_raw", self.counter, self.map_frame, data.prob/2, 1.0, 0.0, 0.0)
		self.marker_pub.publish(marker)

		# Add door to list of doors
		if data.objClass == 'door':
			print 'Processing door.\n'
			ObjectMarker.clusterObject(self.doors, data.x, data.y)

	# Publishes all the doors in the list as markers if the number of doors near that position is greater than a give threshold
	def publishDoorMarkers(self, thresh):
		for door in self.doors:
			if door.count >= thresh:
				marker = ObjectMarker.getMarker(door.x, door.y, 'door', door.id, self.map_frame, 0.3, 0.0, 0.0, 1.0, 0.6)
				self.marker_pub.publish(marker)

	# clusterObject:
	#	Cluster's new object to objects of a list together if the are in range or add a new object to given list
	@staticmethod
	def clusterObject(objList, x, y):
		added = False

		for obj in objList:
			if np.sqrt(np.power(obj.x - x, 2)+np.power(obj.y - y,2)) <= ObjectMarker.max_obj_radii:
				added = True
				obj.addObservation(x, y)

		if not added:
			objList.append(ObjectCounter(x, y))

	# markWorldObject:
	#	Publishes the WorldObject in the specified frame (map) as a Marker (so it can be visualized)
	@staticmethod
	def getMarker(x, y, namespace, id, frame, size, R,G,B, lifeTime=5.0):
		marker = Marker()
		marker.header.stamp = rospy.Time.now()		

		# Frame (map)
		marker.header.frame_id = frame

		# Object type
		marker.ns = namespace

		# Marker identifier
		marker.id = id

		# Sphere
		marker.type = Marker.SPHERE
		marker.action = Marker.ADD

		# Position
		marker.pose.position.x = x
		marker.pose.position.y = y
		marker.pose.position.z = 0.6
		marker.pose.orientation.x = 0.0
		marker.pose.orientation.y = 0.0
		marker.pose.orientation.z = 0.0
		marker.pose.orientation.w = 1.0

		# Size
		marker.scale.x = size
		marker.scale.y = size
		marker.scale.z = size

		# Color
		marker.color.a = 1.0 
		marker.color.r = R
		marker.color.g = G
		marker.color.b = B

		# Lifetime
		marker.lifetime = rospy.Duration(lifeTime)

		return marker


def main(args):

	global objects_topic
	global markers_topic
	global map_frame
	global marker_thresh

	# Initialize the marker object
	om = ObjectMarker(objects_topic, markers_topic, map_frame)

	# Initialize node
	rospy.init_node('object_marker', anonymous=True)

	rate = rospy.Rate(10)

	while not rospy.is_shutdown(): 
		om.publishDoorMarkers(marker_thresh)
		rate.sleep()

if __name__ == '__main__':
	main(sys.argv)
