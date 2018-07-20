#!/usr/bin/env python
""" cmd_pub_m.py
This script publishes the messages it receives at a rate of 0.1
seconds. The drone does not stop after letting go of the key.

Set topic stream through the cmd_vel_topic topic
by default it uses bebop/cmd_vel_set.
"""

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class rep_pub_class(object):
    # Subscribes to cmd_vel_topic, creates a dictionary of topics (self.topic_dict) with one topic set as the active topic (self.topic).
    def __init__(self):
        self.msg = Twist()
        #cmd_vel_topic is for specifying the current active topic
        rospy.Subscriber('cmd_vel_topic', String, self.update_topic)
        #default topic is cmd_vel_set
        cmd_vel_set = rospy.Subscriber('bebop/cmd_vel_set', Twist, self.callback, callback_args = 'bebop/cmd_vel_set')
        self.topic_dict = { 'bebop/cmd_vel_set': cmd_vel_set}
        self.topic = 'bebop/cmd_vel_set'
        # topic read by the bebop
        pub = rospy.Publisher("bebop/cmd_vel", Twist, queue_size=10)
        # publish at 100ms intervals
        while not rospy.is_shutdown():
            rate = rospy.Rate(10)
            pub.publish(self.msg)
            rate.sleep()

    def update_topic(self, data)
        # If the subscriber does not exist, instantiate and add to the dictionary.
        if not (data.data in self.topic_dict):
            self.topic_dict[data.data] = rospy.Subscriber(data.data, Twist, self.callback, data.data)
        self.topic = data.data

    def callback(self, msg, topic):
        """Updates the message being published if that topic is the desired one"""
        if self.topic == topic:
            self.msg = msg

def rep_pub():
    """Sets up node, instantiates the class, keeps thread alive"""
    rospy.init_node('cmd_pub_m')
    rep_pub_class()
    rospy.spin()

if __name__ == "__main__":
    try:
        rep_pub()
    finally:
        pass
