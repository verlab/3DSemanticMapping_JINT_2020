#!/usr/bin/env python

from roslib.message import get_message_class
import rospy
import rosgraph

class DownPublisher():
    def __init__(self):
        rospy.init_node('downsampling_publish', anonymous=True)
        
        prefix = rospy.get_param('~prefix', "/downsampling")
        rate = rospy.get_param('~rate', 1)
        r = rospy.Rate(rate)
        
        self.topics_selected = rospy.get_param('/topics', '')
        self.topics_ = dict({})                  

        self._master = rosgraph.Master(rospy.get_name())
        self.all_topics_info = self._master.getTopicTypes()

        if len(self.topics_selected) == 0:
            for topic in self.all_topics_info:
                self.topics_selected.append(topic[0])

        for topic in self.topics_selected:
            msg_name = [ty for tp, ty in self.all_topics_info if tp == topic][0]
            sub_ = rospy.Subscriber(topic, get_message_class(msg_name), callback = self.callback, callback_args = topic)
            pub_ = rospy.Publisher(prefix+topic, get_message_class(msg_name), queue_size=1)
            msg_ = get_message_class(msg_name)
            self.topics_[topic] = [sub_, pub_, msg_]
    
        rospy.loginfo("Starting Downsamplig Publisher at " + str(rate) + " Hz")
        rospy.loginfo("Topics: " + str(self.topics_selected))
       
        while not rospy.is_shutdown():
            for topic in self.topics_selected:
                self.topics_[topic][1].publish(self.topics_[topic][2])
            r.sleep()
           
    def callback(self, msg, topic):
        self.topics_[topic][2] = msg
        
if __name__ == '__main__':
    try:
        d = DownPublisher()
        rospy.spin()
    except rospy.ROSInterruptException: pass

