#!/usr/bin/env python
# license removed for brevity
from ctypes import ArgumentError
import rospy
import sys
from std_msgs.msg import String


def check_message_validity(message: str):
    avail_comm=['start', 'stop']
    message = message.lower().strip()
    if message not in avail_comm:
        raise ArgumentError('The only allowed arguments are '+ str(avail_comm))
    return message  
        


def talker(message):
    pub = rospy.Publisher('/human_interaction', String, queue_size=1)
    rospy.init_node('human_inter', anonymous=True)
    rospy.loginfo(message)
    pub.publish(message)


if __name__ == '__main__':

    try:
        message = check_message_validity(sys.argv[1])
        talker(message)
    except rospy.ROSInterruptException:
        pass