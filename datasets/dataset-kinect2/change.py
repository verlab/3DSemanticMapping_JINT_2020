#!/usr/bin/python
"""
Copyright (c) 2012,
Systems, Robotics and Vision Group
University of the Balearican Islands
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of Systems, Robotics and Vision Group, University of
      the Balearican Islands nor the names of its contributors may be used to
      endorse or promote products derived from this software without specific
      prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""


PKG = 'bag_tools' # this package name

import roslib
import rospy
import rosbag
import os
import sys
import argparse

def remove_tf(inbag,outbag,frame_ids):
  rospy.loginfo('   Processing input bagfile: %s', inbag)
  rospy.loginfo('  Writing to output bagfile: %s', outbag)
  rospy.loginfo('         Removing frame_ids: %s', ' '.join(frame_ids))

  outbag = rosbag.Bag(outbag,'w')
  frame_parent = ['base_link']
  frame_child = ['kinect2_link']

  for topic, msg, t in rosbag.Bag(inbag,'r').read_messages():
      if topic == "/tf":
          new_transforms = []
          for transform in msg.transforms:
              if transform.header.frame_id not in frame_parent and transform.child_frame_id not in frame_child:
                  new_transforms.append(transform)
          msg.transforms = new_transforms
      outbag.write(topic, msg, t)
  rospy.loginfo('Closing output bagfile and exit...')
  outbag.close();

if __name__ == "__main__":
  rospy.init_node('remove_tf')
  parser = argparse.ArgumentParser(
      description='removes all transforms from the /tf topic that contain one of the given frame_ids in the header as parent or child.')
  parser.add_argument('-i', metavar='INPUT_BAGFILE', required=True, help='input bagfile')
  parser.add_argument('-o', metavar='OUTPUT_BAGFILE', required=True, help='output bagfile')
  parser.add_argument('-f', metavar='FRAME_ID', required=True, help='frame_id(s) of the transforms to remove from the /tf topic', nargs='+')
  args = parser.parse_args()

  try:
    remove_tf(args.i,args.o,args.f)
  except Exception, e:
    import traceback
    traceback.print_exc()
