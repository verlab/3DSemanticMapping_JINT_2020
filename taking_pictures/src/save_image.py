#!/usr/bin/env python
import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

#from __future__ import print_function

image = None

class image_converter:

  def __init__(self):
    #self.image_pub = rospy.Publisher("image_topic_2",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback)

  def callback(self,data):
    global image
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      image = cv_image
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape
    #print "shape: %s" % cv_image.shape
    
    #if cols > 60 and rows > 60 :
    #  cv2.circle(cv_image, (50,50), 10, 255)

    #cv2.imshow("Image window", image)
    #cv2.waitKey(3)

    #try:
    #  self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    #except CvBridgeError as e:
    #  print(e)

def main(args):
  global image
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  running = True
  number = 100
  while  not rospy.is_shutdown(): 
    try:
      raw_input('Press enter, take image. ')
      # Save image
      name = 'door_%d.jpg' % number
      number += 1
      cv2.imwrite(name, image)
      print 'image saved'
      #rospy.spin()
    except KeyboardInterrupt:
      print("Shutting down")
      break


if __name__ == '__main__':
    main(sys.argv)