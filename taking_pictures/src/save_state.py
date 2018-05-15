#!/usr/bin/env python
import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
import tf
import numpy as np

from nav_msgs.msg import MapMetaData
from nav_msgs.msg import OccupancyGrid
from custom_msgs.msg import WorldObject

#from __future__ import print_function

map_topic = 'map'
objects_topic = 'objects_filtered'
robot_frame = 'base_link'
map_frame = 'map'

class state_saver:

  def __init__(self, map_topic, objects_topic, robot_frame, map_frame):
    self.count = 0 # Number of saved states
    self.objects_topic = objects_topic
    self.robot_frame = robot_frame
    self.map_frame = map_frame

    self.map = OccupancyGrid()
    self.listener = tf.TransformListener()
    
    self.objects = []

    self.map_sub = rospy.Subscriber(map_topic, OccupancyGrid, self.map_callback)
    self.objects_sub = rospy.Subscriber(objects_topic, WorldObject, self.objects_callback)
  
  def getMapArray(self):
      for i in range(len(self.map.data)):
          if self.map.data[i] == -1:
            self.map.data[i] = 126
          else :
            self.map.data[i] = int(255-(self.map.data[i]/100.0)*255)
              
      return np.asarray(self.map.data).reshape((self.map.info.height, self.map.info.width))

  def getCellPosFromWorldPos(self, world_x, world_y):
    offsetX = world_x - self.map.info.origin.position.x
    offsetY = world_y - self.map.info.origin.position.y
    cell_x = int(round(offsetX/self.map.info.resolution))
    cell_y = int(round(offsetY/self.map.info.resolution))

    return cell_x, cell_y

  def map_callback(self,data):
    self.map = data
    self.map.data = list(data.data)

  def objects_callback(self, data):
    added = False
    for i in range(len(self.objects)):
      if self.objects[i].objClass == data.objClass and self.objects[i].prob == data.prob:
        added = True
        break
    if not added:
      print 'Added new object!\n'
      self.objects.append(data)

  def save_state(self):
    
    self.count += 1

    # Robot position
    try:
      now = rospy.Time.now() 
      self.listener.waitForTransform(self.robot_frame, self.map_frame, now, rospy.Duration(1.0))
      (trans,rot) = self.listener.lookupTransform(self.robot_frame, self.map_frame, rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      print 'Lookup error robot position'

    robot_x = trans[0]
    robot_y = trans[1]
    map_x, map_y = self.getCellPosFromWorldPos(robot_x, robot_y)

    print "Robot position: (%f, %f) m; [%d, %d] pixels.\n" % (robot_x, robot_y, map_x, map_y)
    print "Objects: \n"
    for i in range(len(self.objects)):
      obj_x, obj_y = self.getCellPosFromWorldPos( self.objects[i].x, self.objects[i].y)
      print "Class: %s, position: (%f, %f) m, angle: %f rad; [%d, %d] pixels \n" % (self.objects[i].objClass, self.objects[i].x, self.objects[i].y, self.objects[i].angle, obj_x, obj_y)

    print "Map: %d, %d" % ((self.map.info.height, self.map.info.width))

    map_image = self.getMapArray()
    im_name = 'map'+str(self.count)+'.png'
    cv2.imwrite(im_name, map_image)
    
def main(args):
  global map_topic, objects_topic, robot_frame, map_frame

  rospy.init_node('state_saver', anonymous=True)
  ss = state_saver(map_topic, objects_topic, robot_frame, map_frame)

  while  not rospy.is_shutdown(): 
    try:
      ina = raw_input('Press enter, take image. (q to exit)\n')
      if ina == 'q' or ina == 'exit':
          break
      ss.save_state()
      
      #rospy.spin()
    except KeyboardInterrupt:
      print("Shutting down")
      break


if __name__ == '__main__':
    main(sys.argv)