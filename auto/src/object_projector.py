#!/usr/bin/env python

# objet_projector.py :
#	 Subscribes to the darknet bouding boxes topic and publishes the detected objects in the map as markers

import rospy
import roslib
import sys
import tf
import numpy as np
import struct
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from darknet_ros_msgs.msg import BoundingBoxes
from darknet_ros_msgs.msg import BoundingBox

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
darkent_boxes_topic = '/darknet_ros/bounding_boxes'
pointcloud_topic = '/camera/depth/points'
cmd_vel_topic = '/navigation_velocity_smoother/raw_cmd_vel'

# TOPICS [out]
objects_topic = '/detected_objects_raw'

# FRAMES
map_frame = 'map'

#
#######################################################################

class ObjectProjector:
	def __init__(self, detected_boxes_topic, pointcloud_topic, cmd_vel_topic, objects_topic, map_frame):
		self.detected_boxes_topic = detected_boxes_topic
		self.pointcloud_topic = pointcloud_topic
		self.cmd_vel_topic = cmd_vel_topic
		self.objects_topic = objects_topic
		self.map_frame = map_frame

		self.listener = None
		self.sample_point_count = 20
		self.cloud = PointCloud2()
		self.can_detect = True

		# SUBSCRIBERS
		self.boxes_sub = rospy.Subscriber(detected_boxes_topic, BoundingBoxes, self.boundingBoxCallback)
		self.cloud_sub = rospy.Subscriber(pointcloud_topic, PointCloud2, self.pointcloudCallback)
		self.cmd_sub = rospy.Subscriber(cmd_vel_topic, Twist, self.cmdCallback)

		# PUBLISHERS
		self.objects_pub = rospy.Publisher(objects_topic, WorldObject, queue_size=10)

	def initListener(self):
		# Set listener AFTER node initialized
		self.listener = tf.TransformListener()

	def pointcloudCallback(self, data):
		# Data is in camera_link frame (or something like that). Couldn't convert to [map] frame because the python API only supports pointcloud format.
		self.cloud = data
		#print 'Received cloud. Size: %dx%d\n' % (self.cloud.width, self.cloud.height)

	def pixelTo3DPoint(self, cloud, u, v):
		width = cloud.width
		height = cloud.height
		point_step = cloud.point_step
		row_step = cloud.row_step

		array_pos = v*row_step + u*point_step
		#print data[array_pos]

		bytesX = [ord(x) for x in cloud.data[array_pos:array_pos+4]]
		bytesY = [ord(x) for x in cloud.data[array_pos+4: array_pos+8]]
		bytesZ = [ord(x) for x in cloud.data[array_pos+8:array_pos+12]]

		byte_format=struct.pack('4B', *bytesX)
		X = struct.unpack('f', byte_format)[0]

		byte_format=struct.pack('4B', *bytesY)
		Y = struct.unpack('f', byte_format)[0]

		byte_format=struct.pack('4B', *bytesZ)
		Z = struct.unpack('f', byte_format)[0]

		return Point(X, Y, Z)

	def transformPointFrame(self, point, curr_frame, target_frame):
		ps = PointStamped()
		ps.header.frame_id = curr_frame
		ps.header.stamp = rospy.Time(0)
		ps.point = point
		
		res = [np.nan, np.nan, np.nan]                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
		#print 'waiting for transform from %s to %s' % (target_frame, ps.header.frame_id)

		now = rospy.Time.now()
		self.listener.waitForTransform(ps.header.frame_id, target_frame, now, rospy.Duration(5.0))
		#listener.waitForTransform(ps.header.frame_id, target_frame, rospy.Time.now(), rospy.Time(2.0))

		#rospy.sleep(1.0)
		#ps.header.stamp = rospy.Time.now()
		t = self.listener.transformPoint(target_frame, ps)
		res = t.point

		detector_published = True
		return res

	def randomPoints(self, xmin, xmax, ymin, ymax, count):
		result = []
		xoffset = abs(xmax-xmin)
		yoffset = abs(ymax-ymin)
		for i in xrange(count):
			x = xmin+int(xoffset*np.random.uniform(0,1))
			y = ymin+int(yoffset*np.random.uniform(0,1))
			result.append([x,y])

		return result

	# Main method: get the published bounding boxes and transform to position in map.
	def boundingBoxCallback(self, data):
		frame = self.cloud.header.frame_id
		#boxes = data.boundingBoxes
		boxes = data.bounding_boxes

		for box in boxes:
			obj = WorldObject()

			prob = box.probability
			xmin = box.xmin
			xmax = box.xmax
			ymin = box.ymin
			ymax = box.ymax

			print "Found object (class %s) ! xmin: %d xmax: %d ymin: %d ymax: %d\n" % (box.Class, xmin, xmax, ymin, ymax)

			if not self.can_detect:
				continue

			xcenter = int((xmin+xmax)/2.0)
			ycenter = int((ymin+ymax)/2.0)
			offset = 15

			# Sample random points from inside box 
			im_pixels = self.randomPoints(xcenter-offset, xcenter+offset, ycenter-offset, ycenter+offset, self.sample_point_count)

			# For each point, get the XYZ coord
			points_3d = []
			final_points = []

			for u,v in im_pixels:
				point_3d = self.pixelTo3DPoint(self.cloud, u, v)
				
				if not (np.isnan(point_3d.x) or point_3d.x == np.inf):
					points_3d.append(point_3d)

			if len(points_3d) < 3:
				print "No depth data available!\n"
				continue

			# Select points and transform them into map frame
			for p in points_3d:

				# Convert to point in [map] frame
				point = self.transformPointFrame(p, frame, map_frame)

				# Save point in final points
				final_points.append(point)

			# Calculate average position
			Xs = [p.x for p in final_points]
			Ys = [p.y for p in final_points]
			avgX = sum(Xs)*1.0/len(Xs)
			avgY = sum(Ys)*1.0/len(Ys)

			# Publish object
			obj.objClass = box.Class
			obj.x = avgX
			obj.y = avgY
			obj.prob = prob
			self.objects_pub.publish(obj)

	def cmdCallback(self, data):
		w = data.angular.z
		if abs(w) > 0.15:
			self.can_detect = False
		else:
			self.can_detect = True

def main(args):
	global darkent_boxes_topic
	global pointcloud_topic
	global cmd_vel_topic
	global image_out_topic
	global objects_topic
	global map_frame

	# Initialize object projector
	op = ObjectProjector(darkent_boxes_topic, pointcloud_topic, cmd_vel_topic, objects_topic, map_frame)

	# Initialize node
	rospy.init_node('object_projector', anonymous=True)

	# Initialize listener after initializing node
	op.initListener()

	try:
		# spin : wait for program to pause
		rospy.spin()

	except KeyboardInterrupt:
		print("Shutting down")

if __name__ == '__main__':
	main(sys.argv)
