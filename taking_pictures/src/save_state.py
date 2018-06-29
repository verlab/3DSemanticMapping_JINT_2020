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
objects_raw_topic = 'objects_raw'
robot_frame = 'base_link'
map_frame = 'map'

raw_size = 3
path_size = 4
line_length = 19.0
line_thickness = 9

class state_saver:

  def __init__(self, map_topic, objects_topic, objects_raw_topic, robot_frame, map_frame):
    self.count = 0 # Number of saved states
    self.objects_topic = objects_topic
    self.robot_frame = robot_frame
    self.map_frame = map_frame

    self.map = OccupancyGrid()
    self.listener = tf.TransformListener()
    
    self.objects = []
    self.objects_raw = []
    self.robot_pos = []

    self.map_sub = rospy.Subscriber(map_topic, OccupancyGrid, self.map_callback)
    self.objects_sub = rospy.Subscriber(objects_topic, WorldObject, self.objects_callback)
    self.objects_raw_sub = rospy.Subscriber(objects_raw_topic, WorldObject, self.objects_raw_callback)
  
  def getMapArray(self):
      map_copy = list(self.map.data)
      for i in range(len(map_copy)):
          if map_copy[i] == -1:
            map_copy[i] = 126
          else :
            map_copy[i] = int(255-(map_copy[i]/100.0)*255)
            #map_copy[i] = int((map_copy[i]/100.0)*255)
              
      return np.asarray(map_copy).reshape((self.map.info.height, self.map.info.width)).astype(np.uint8)

  def getCellPosFromWorldPos(self, world_x, world_y):
    offsetX = world_x - self.map.info.origin.position.x
    offsetY = world_y - self.map.info.origin.position.y
    cell_y = 0
    cell_x = 0

    if np.isnan(offsetX) or np.isnan(offsetY):
      cell_x = 0
      cell_y = 0
    else:
      cell_x = int(round(offsetX/self.map.info.resolution))
      cell_y = int(round(offsetY/self.map.info.resolution))

    return cell_x, cell_y

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

  def save_robot_pos(self):
    # Robot position
    try:
      now = rospy.Time.now() 
      #self.listener.waitForTransform(self.robot_frame, self.map_frame, now, rospy.Duration(1.0))      
      #(trans,rot) = self.listener.lookupTransform(self.robot_frame, self.map_frame, rospy.Time(0))
      self.listener.waitForTransform(self.map_frame, self.robot_frame, now, rospy.Duration(1.0))
      (trans,rot) = self.listener.lookupTransform(self.map_frame, self.robot_frame, rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      print 'Lookup error robot position'
      return 

    except Exception:
      print 'No frame found yet. '
      return 

    robot_x = trans[0]
    robot_y = trans[1]
    quaternion = (rot[0], rot[1], rot[2], rot[3])
    euler = tf.transformations.euler_from_quaternion(quaternion)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]

    # Apply rotation ... 
    rot = [[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]]
    #trans = np.matmul()

    print 'trans: %f %f' % (robot_x, robot_y)
    #print 'rot: %f %f %f \n' % (roll, pitch, yaw)
    print 'rot: %f %f %f %f\n' % (quaternion[0], quaternion[1], quaternion[2], quaternion[3])

    self.robot_pos.append((robot_x, robot_y))

  def save_all(self):
    map_image = self.getMapArray()
    map_marked = self.getMapArray()

    map_image = cv2.cvtColor(map_image,cv2.COLOR_GRAY2RGB)
    map_marked = cv2.cvtColor(map_marked,cv2.COLOR_GRAY2RGB)

    im_name = 'slam_map.png'
    marked_name = 'maked_map.png'

    # Mark robot path
    for i in range(len(self.robot_pos)):
      x_meters, y_meters = self.robot_pos[i]
      x, y = self.getCellPosFromWorldPos(x_meters, y_meters)
      print 'marking robot at %d %d ' % (x, y)
      cv2.circle(map_marked, (x,y), path_size, (0,0,220), -1)

    # Mark raw objects
    for i in range(len(self.objects_raw)):
      x_meters, y_meters = (self.objects_raw[i].x ,self.objects_raw[i].y) 
      if np.isnan(x_meters) or np.isnan(y_meters) :
        print "NaN value found\n" 
        continue
      x, y = self.getCellPosFromWorldPos(x_meters, y_meters)
      print 'marking raw obj at %d %d ' % (x, y)      
      cv2.circle(map_marked, (x,y), raw_size, (150,0,150), -1)

    # Mark filtered objects
    print 'Making Objects: \n'
    for i in range(len(self.objects)):
      x_meters, y_meters = (self.objects[i].x ,self.objects[i].y) 
      if np.isnan(x_meters) or np.isnan(y_meters) :
        print "NaN value found\n" 
        continue

      x, y = self.getCellPosFromWorldPos(x_meters, y_meters)
      a = self.objects[i].angle+1.57
      #print 'marking obj at %d %d %f' % (x, y, a) 
      print '%f %f' % (x_meters, y_meters)
      offsetx = int(round(line_length*np.cos(a)))
      offsety = int(round(line_length*np.sin(a)))
      cv2.line(map_marked, (x-offsetx, y-offsety), (x+offsetx, y+offsety), (0,255,0), line_thickness)

    cv2.imwrite(im_name, map_image)
    cv2.imwrite(marked_name, map_marked)

# Callbacks
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

  def objects_raw_callback(self, data):
    self.objects_raw.append(data)

def main(args):
  global map_topic, objects_topic, robot_frame, map_frame, objects_raw_topic

  rospy.init_node('state_saver', anonymous=True)
  ss = state_saver(map_topic, objects_topic, objects_raw_topic, robot_frame, map_frame)
  rate = rospy.Rate(1)

  while  not rospy.is_shutdown(): 
    try:
      print("Saving robot position. \n")
      ss.save_robot_pos()
      rate.sleep()

    except KeyboardInterrupt:
      print("Keyboard exception. \n\n\n")
      break
  
  print("Saving all and terminating. \n\n\n")
  ss.save_all()
  

if __name__ == '__main__':
    main(sys.argv)
