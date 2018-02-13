#!/usr/bin/env python

# image_publisher.py :
# 	Receives image data from camera and publishes to darkent with a (possibly) lower frequency, to minimize CPU usage.

import rospy
import sys
import roslib
from sensor_msgs.msg import Image

#######################################################################
# PARAMETERS 
# TODO: parametrize these as inputs from launch file

# Publish Frequency
publish_frequency = 0.6

# TOPICS [in]
image_in_topic = '/input'

# TOPICS [out]
image_out_topic = '/output'

#
#######################################################################


class ImagePublisher:
	def __init__(self, in_topic, out_topic):

		# Image that will be overwritten by the subscriber
		self.image = Image()

		# SUBSCRIBERS
		# Image subscriber (gathers camera data)
		self.image_sub = rospy.Subscriber(in_topic, Image, self.imageCallback)

		# Image publisher (publish the gathered image from darknet at a lower rate)
		self.image_pub = rospy.Publisher(out_topic, Image, queue_size=1)


	def imageCallback(self, data):
		self.image = data

	def publishImage(self):
		self.image_pub.publish(self.image)

def main(args):
	global publish_frequency
	global image_in_topic
	global image_out_topic

	ipub = ImagePublisher(image_in_topic, image_out_topic)

	# Initialize node
	rospy.init_node('image_publisher', anonymous=True)

	rate = rospy.Rate(publish_frequency)

	while not rospy.is_shutdown(): 
		rate.sleep()

		# Publish image to detector
		ipub.publishImage()

if __name__ == '__main__':
	main(sys.argv)
