#!/usr/bin/env python

import rospy
import roslib
import tf
import sys
import math
import numpy as np
import struct
import copy
from visualization_msgs.msg import Marker

from custom_msgs.msg import WorldObject

from filtered_instances import FilteredInstances

# TOPICS [in]
objects_topic_raw = '/objects_raw'

# TOPICS [out]
objects_topic_filtered = '/objects_filtered'

# FILTER 
radius = 0.9
process_cov = 0.3
meas_cov = 5

# Debug
markers_topic = '/markers'

doors = None

def object_callback(data):
	doors.addMeasurement((data.x,data.y, data.angle)) 
	# print 'Added door. \n'

def getMarker(x, y, angle, namespace, id, frame, size=0.4, R=1.0,G=0.0,B=0.0, lifeTime=5.0):
	marker = Marker()
	marker.header.stamp = rospy.Time.now()		

	# Frame (map)
	marker.header.frame_id = frame

	# Object type
	marker.ns = namespace

	# Marker identifier
	marker.id = id

	# Sphere
	marker.type = Marker.CUBE
	marker.action = Marker.ADD

	# Size
	marker.scale.x = 0.05
	marker.scale.y = size
	marker.scale.z = 2.0

	# Position
	q = tf.transformations.quaternion_from_euler(0, 0, angle)
	marker.pose.position.x = x
	marker.pose.position.y = y
	marker.pose.position.z = marker.scale.z/2.0
	marker.pose.orientation.x = q[0]
	marker.pose.orientation.y = q[1]
	marker.pose.orientation.z = q[2]
	marker.pose.orientation.w = q[3]

	# Color
	marker.color.a = 1.0 
	marker.color.r = R
	marker.color.g = G
	marker.color.b = B

	# Lifetime
	marker.lifetime = rospy.Duration(lifeTime)

	return marker

def getMarkerArrow(x, y, angle, namespace, id, frame, size=0.4, R=1.0,G=0.0,B=0.0, lifeTime=5.0):
	marker = Marker()
	marker.header.stamp = rospy.Time.now()		

	# Frame (map)
	marker.header.frame_id = frame

	# Object type
	marker.ns = namespace

	# Marker identifier
	marker.id = id

	# Sphere
	marker.type = Marker.ARROW
	marker.action = Marker.ADD

	q = tf.transformations.quaternion_from_euler(0, 0, angle)

	# Position
	marker.pose.position.x = x
	marker.pose.position.y = y
	marker.pose.position.z = 0.6
	marker.pose.orientation.x = q[0]
	marker.pose.orientation.y = q[1]
	marker.pose.orientation.z = q[2]
	marker.pose.orientation.w = q[3]

	# Size
	marker.scale.x = size
	marker.scale.y = 0.05
	marker.scale.z = 0.05

	# Color
	marker.color.a = 1.0 
	marker.color.r = R
	marker.color.g = G
	marker.color.b = B

	# Lifetime
	marker.lifetime = rospy.Duration(lifeTime)

	return marker

def main(args):

	global objects_topic_raw, doors, radius, process_cov, meas_cov 

	# Initialize node
	rospy.init_node('object_marker', anonymous=True)
	rate = rospy.Rate(5) 

	rospy.Subscriber(objects_topic_raw, WorldObject, object_callback)
	obj_pub = rospy.Publisher(objects_topic_filtered, WorldObject, queue_size=10)

	marker_pub = rospy.Publisher(markers_topic, Marker, queue_size=10)

	doors = FilteredInstances(radius, process_cov, meas_cov)

	while not rospy.is_shutdown(): 
		
		# Publish filtered objects
		for i in range(len(doors.instances)):
			pred = doors.predictions[i]
			obj_filtered = WorldObject()
			obj_filtered.objClass = 'door'
			obj_filtered.x = pred[0]
			obj_filtered.y = pred[1]
			obj_filtered.angle = doors.angles[i]
			obj_filtered.prob = float(i)
			
			if doors.observations[i] > 1.0:
				obj_pub.publish(obj_filtered)

				# Publish marker
				#marker = getMarker(obj_filtered.x, obj_filtered.y, obj_filtered.angle, 'door', i, 'map', 0.4, doors.colors[i][0]/255.0, doors.colors[i][1]/255.0, doors.colors[i][2]/255.0)
				marker = getMarker(obj_filtered.x, obj_filtered.y, obj_filtered.angle, 'door', i, 'map', 0.8, 0.0, 1.0, 0.0)
				marker_pub.publish(marker)


		rate.sleep()

if __name__ == '__main__':
	main(sys.argv)
