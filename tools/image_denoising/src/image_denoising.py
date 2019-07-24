#!/usr/bin/env python

from roslib.message import get_message_class
import rospy
import rosgraph
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv
import numpy as np

VERBOSE=False

class NoisePublisher():
    def __init__(self):
        rospy.init_node('image_denoising', anonymous=True)
        
        prefix = rospy.get_param('~prefix', "/denoised")
        self.noise = rospy.get_param('~noise')
        rate = 30  
        r = rospy.Rate(rate)    
        self.topics_selected = rospy.get_param('/topics', '')
        self.topics_ = dict({})                  

        self._master = rosgraph.Master(rospy.get_name())
        self.all_topics_info = self._master.getTopicTypes()

        if len(self.topics_selected) == 0:
            rospy.loginfo("No topic selected, please add camera rgb or depth topics") 
        if VERBOSE :
            print "subscribed to /camera/rgb/image_raw"               
	self.bridge = CvBridge()
        for topic in self.topics_selected:
            msg_name = [ty for tp, ty in self.all_topics_info if tp == topic][0]
            self.pub_ = rospy.Publisher(prefix+topic,Image,queue_size=1)
            sub_ = rospy.Subscriber(topic, get_message_class(msg_name), callback = self.callback, callback_args = topic)            
            #msg_ = Image
            #self.topics_[topic] = [sub_, pub_, msg_]
    
        rospy.loginfo("Topics with noise: std = " + str(self.noise))
        
       
        #while not rospy.is_shutdown():
            #for topic in self.topics_selected:
            #    pub_.publish(self.topics_[topic][2])
            #r.sleep()
           
    def callback(self, msg, topic):
        #self.topics_[topic][2] = msg
        try:
           cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
           print(e)
        row,col,ch = cv_image.shape
        mean = 0
        var = self.noise
        sigma = var**0.5
        gauss = np.random.normal(mean,sigma,(row,col,ch))
        gauss = gauss.reshape(row,col,ch)
        cv_image = cv_image.astype(np.float)
        noisy = cv_image + gauss
        #msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        noisy = noisy.astype(np.uint8)
        self.pub_.publish(self.bridge.cv2_to_imgmsg(noisy, "bgr8"))
        
if __name__ == '__main__':
    try:
        d = NoisePublisher()
        rospy.spin()
    except rospy.ROSInterruptException: pass


