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
from custom_msgs.msg import ObjectList
from rtabmap_ros.msg import MapGraph

from filtered_instances import FilteredInstances

# TOPICS [in]
object_list_topic_raw = '/objects_raw/list'
graph_list = '/rtabmap/mapGraph'

# TOPICS [out]
objects_topic_filtered = '/objects_filtered'

# Perform graph node update 
doGraphUpdate = False
printCovariances = False
printPositions = True

# FILTER 
process_cov = 0.3
meas_cov = 5

# Classes 
doors = None 
benches = None
trashes = None
fires = None
waters = None

# Association threshold 
door_radius = 3
bench_radius = 4
trash_radius = 3.4
fire_radius = 2.7
water_radius = 2.7

# Debug
markers_topic = '/markers'

def object_list_callback(object_list):
	door_list = []
	bench_list = []
	trash_list = []
	fire_list = []
	water_list = []
	for obj in object_list.objects:
		if obj.objClass == 'door':
			door_list.append((obj.x,obj.y, obj.angle)) 

		elif obj.objClass == 'bench':
			bench_list.append((obj.x,obj.y, obj.angle))

		elif obj.objClass == 'trash' :
			trash_list.append((obj.x,obj.y, obj.angle))

		elif obj.objClass == 'fire':
			fire_list.append((obj.x,obj.y, obj.angle))

		elif obj.objClass == 'water': 
			water_list.append((obj.x,obj.y, obj.angle))

	doors.addMeasurementList(door_list)
	benches.addMeasurementList(bench_list)
	trashes.addMeasurementList(trash_list)
	fires.addMeasurementList(fire_list)
	waters.addMeasurementList(water_list)

def graph_list_callback(graph_list):
	global doors, benches, trashes, fires, waters 
	doors.updateGraphList(graph_list.poses, graph_list.posesId)
	benches.updateGraphList(graph_list.poses, graph_list.posesId)
	trashes.updateGraphList(graph_list.poses, graph_list.posesId)
	fires.updateGraphList(graph_list.poses, graph_list.posesId)
	waters.updateGraphList(graph_list.poses, graph_list.posesId)

def getTextMarker(label, x, y, height, namespace, id, frame, size, R, G, B, lifeTime):
	marker = Marker()
	marker.header.stamp = rospy.Time.now()		

	# Frame (map)
	marker.header.frame_id = frame

	# Object type
	marker.ns = namespace+"_label"

	# Marker identifier
	marker.id = id

	marker.text = label

	# Text
	marker.type = Marker.TEXT_VIEW_FACING
	marker.action = Marker.ADD

	# Size
	marker.scale.z = size

	# Position
	marker.pose.position.x = x
	marker.pose.position.y = y
	marker.pose.position.z = height

	# Color
	marker.color.a = 1.0 
	marker.color.r = R
	marker.color.g = G
	marker.color.b = B

	# Lifetime
	marker.lifetime = rospy.Duration(lifeTime)

	return marker

def getMarker(x, y, z, angle, namespace, id, frame, size_x=0.4, size_y=0.4, size_z=0.4, R=1.0,G=0.0,B=0.0, lifeTime=5.0):
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
	marker.scale.x = size_x
	marker.scale.y = size_y
	marker.scale.z = size_z

	# Position
	q = tf.transformations.quaternion_from_euler(0, 0, angle)
	marker.pose.position.x = x
	marker.pose.position.y = y
	marker.pose.position.z = z
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
	
	global object_list_topic_raw, object_list_topic_raw, process_cov, meas_cov, doGraphUpdate
	global doors, benches, trashes, fires, waters 
	global door_radius, bench_radius, trash_radius, fire_radius, water_radius

	# Initialize node
	rospy.init_node('object_marker', anonymous=True)
	rate = rospy.Rate(5) 

	rospy.Subscriber(object_list_topic_raw, ObjectList, object_list_callback)
	if doGraphUpdate:
		rospy.Subscriber(graph_list, MapGraph, graph_list_callback)

	obj_pub = rospy.Publisher(objects_topic_filtered, WorldObject, queue_size=10)
	marker_pub = rospy.Publisher(markers_topic, Marker, queue_size=10)

	# Object instance lists
	doors = FilteredInstances('door', door_radius, process_cov, meas_cov)
	benches = FilteredInstances('bench', bench_radius, process_cov, meas_cov)
	trashes = FilteredInstances('trash bin', trash_radius, process_cov, meas_cov) 
	fires = FilteredInstances('fire extinguisher', fire_radius, process_cov, meas_cov)
	waters = FilteredInstances('water fountain', water_radius, process_cov, meas_cov) 

	while not rospy.is_shutdown(): 
		
		life_time = 5.0
		min_obs = 1.0
		# Publish doors
		for i in range(len(doors.instances)):
			pred = doors.predictions[i]
			obj_filtered = WorldObject()
			obj_filtered.objClass = 'door'
			obj_filtered.x = pred[0]
			obj_filtered.y = pred[1]
			obj_filtered.angle = doors.angles[i]
			obj_filtered.prob = float(i)
			
			if doors.observations[i] > min_obs:
				obj_pub.publish(obj_filtered)

				class_name = 'door'
				height = 0.9
				frame = 'map'
				size_x = 0.05
				size_y = 0.6
				size_z = 1.8
				R = 0.0
				G = 1.0 
				B = 0.0

				# Publish marker
				marker = getMarker(obj_filtered.x, obj_filtered.y, height, obj_filtered.angle, class_name, i, frame, size_x, size_y, size_z, R, G, B, life_time)
				marker_pub.publish(marker)

				text = getTextMarker(class_name, obj_filtered.x, obj_filtered.y, height+size_z/2.0 + 0.5, class_name, i, frame, 0.3, R, G, B, life_time)
				marker_pub.publish(text)

		# Publish benches
		for i in range(len(benches.instances)):
			pred = benches.predictions[i]
			obj_filtered = WorldObject()
			obj_filtered.objClass = 'bench'
			obj_filtered.x = pred[0]
			obj_filtered.y = pred[1]
			obj_filtered.angle = benches.angles[i]
			obj_filtered.prob = float(i)
			
			if benches.observations[i] > min_obs:
				obj_pub.publish(obj_filtered)

				class_name = 'bench'
				height = 0.4
				frame = 'map'
				size_x = 0.4
				size_y = 0.4
				size_z = 0.5
				R = 0.0
				G = 0.1 
				B = 0.8

				# Publish marker
				marker = getMarker(obj_filtered.x, obj_filtered.y, height, obj_filtered.angle, class_name, i, frame, size_x, size_y, size_z, R, G, B, life_time)
				marker_pub.publish(marker)

				text = getTextMarker(class_name, obj_filtered.x, obj_filtered.y, height+size_z/2.0 + 0.5, class_name, i, frame, 0.3, R, G, B, life_time)
				marker_pub.publish(text)

		# Publish trashes
		for i in range(len(trashes.instances)):
			pred = trashes.predictions[i]
			obj_filtered = WorldObject()
			obj_filtered.objClass = 'trash'
			obj_filtered.x = pred[0]
			obj_filtered.y = pred[1]
			obj_filtered.angle = trashes.angles[i]
			obj_filtered.prob = float(i)
			
			if trashes.observations[i] > min_obs:
				obj_pub.publish(obj_filtered)

				class_name = 'trash'
				height = 0.21
				frame = 'map'
				size_x = 0.25
				size_y = 0.25
				size_z = 0.4
				R = 0.8
				G = 0.7 
				B = 0.1

				# Publish marker
				marker = getMarker(obj_filtered.x, obj_filtered.y, height, obj_filtered.angle, class_name, i, frame, size_x, size_y, size_z, R, G, B, life_time)
				marker_pub.publish(marker)

				text = getTextMarker('trash bin', obj_filtered.x, obj_filtered.y, height+size_z/2.0 + 0.5, class_name, i, frame, 0.3, R, G, B, life_time)
				marker_pub.publish(text)

		# Publish fires
		for i in range(len(fires.instances)):
			pred = fires.predictions[i]
			obj_filtered = WorldObject()
			obj_filtered.objClass = 'fire'
			obj_filtered.x = pred[0]
			obj_filtered.y = pred[1]
			obj_filtered.angle = fires.angles[i]
			obj_filtered.prob = float(i)
			
			if fires.observations[i] > min_obs:
				obj_pub.publish(obj_filtered)

				class_name = 'fire'
				height = 1.4
				frame = 'map'
				size_x = 0.3
				size_y = 0.3
				size_z = 0.4
				R = 1.0
				G = 0.1
				B = 0.1

				# Publish marker
				marker = getMarker(obj_filtered.x, obj_filtered.y, height, obj_filtered.angle, class_name, i, frame, size_x, size_y, size_z, R, G, B, life_time)
				marker_pub.publish(marker)

				text = getTextMarker('fire extinguisher', obj_filtered.x, obj_filtered.y, height+size_z/2.0 + 0.5, class_name, i, frame, 0.3, R, G, B, life_time)
				marker_pub.publish(text)

		# Publish waters
		for i in range(len(waters.instances)):
			pred = waters.predictions[i]
			obj_filtered = WorldObject()
			obj_filtered.objClass = 'water'
			obj_filtered.x = pred[0]
			obj_filtered.y = pred[1]
			obj_filtered.angle = waters.angles[i]
			obj_filtered.prob = float(i)
			
			if waters.observations[i] > min_obs:
				obj_pub.publish(obj_filtered)

				class_name = 'water'
				height = 1.0
				frame = 'map'
				size_x = 0.3
				size_y = 0.3
				size_z = 0.5
				R = 0.4
				G = 0.8 
				B = 1.0
				

				# Publish marker
				marker = getMarker(obj_filtered.x, obj_filtered.y, height, obj_filtered.angle, class_name, i, frame, size_x, size_y, size_z, R, G, B, life_time)
				marker_pub.publish(marker)

				text = getTextMarker('water fountain', obj_filtered.x, obj_filtered.y, height+size_z/2.0 + 0.5, class_name, i, frame, 0.3, R, G, B, life_time)
				marker_pub.publish(text)
			
		rate.sleep()

if __name__ == '__main__':
	try: 
		main(sys.argv)
	except rospy.exceptions.ROSInterruptException:
		print 'SHUTTING DOWN'
		
		if(printCovariances):
			num, vx2, vy2, cxy = doors.getMeanCovariance()
			print 'doors: vx2 = '+vx2+', vy2 = '+vy2+', cxy = '+cxy+' ['+num+' instances]'

			num, vx2, vy2, cxy = benches.getMeanCovariance()
			print 'benches: vx2 = '+vx2+', vy2 = '+vy2+', cxy = '+cxy+' ['+num+' instances]'

			num, vx2, vy2, cxy = trashes.getMeanCovariance()
			print 'trashes: vx2 = '+vx2+', vy2 = '+vy2+', cxy = '+cxy+' ['+num+' instances]'

			num, vx2, vy2, cxy = fires.getMeanCovariance()
			print 'fires: vx2 = '+vx2+', vy2 = '+vy2+', cxy = '+cxy+' ['+num+' instances]'

			num, vx2, vy2, cxy = waters.getMeanCovariance()
			print 'waters: vx2 = '+vx2+', vy2 = '+vy2+', cxy = '+cxy+' ['+num+' instances]'
			print '\n'
			
		if(printPositions):

			for i in range(len(doors.instances)):
				pred = doors.predictions[i]
				x = pred[0]
				y = pred[1]
				print 'door '+str(i+1)+' : '+str(x) + ' '+str(y) 
				print '\n'

			for i in range(len(trashes.instances)):
				pred = trashes.predictions[i]
				x = pred[0]
				y = pred[1]
				print 'trash '+str(i+1)+' : '+str(x) + ' '+str(y) 
				print '\n'

			for i in range(len(waters.instances)):
				pred = waters.predictions[i]
				x = pred[0]
				y = pred[1]
				print 'water '+str(i+1)+' : '+str(x) + ' '+str(y) 
				print '\n'

			for i in range(len(benches.instances)):
				pred = benches.predictions[i]
				x = pred[0]
				y = pred[1]
				print 'bench '+str(i+1)+' : '+str(x) + ' '+str(y) 
				print '\n'

			for i in range(len(fires.instances)):
				pred = fires.predictions[i]
				x = pred[0]
				y = pred[1]
				print 'fire '+str(i+1)+' : '+str(x) + ' '+str(y) 
				print '\n'
