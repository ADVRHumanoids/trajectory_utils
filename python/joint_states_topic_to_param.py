#!/usr/bin/env python

import rospy
import sys
from sensor_msgs.msg import JointState

_ok = False

def joint_states_callback(msg):
    global _ok
    rospy.loginfo("received joint_states")
    param_dict = dict()
    for joint_id, joint_name in enumerate(msg.name):
        param_dict[joint_name] = msg.position[joint_id]

    rospy.set_param('zeros', param_dict)
    _ok = True

if __name__ == '__main__':
    """
    simple python node that subscribes to a joint state publisher and transforms every JointState message to a ros zeros parameter
    """

    global _ok

    rospy.myargv(argv=sys.argv)
    rospy.init_node('joint_states_topic_to_param')

    rospy.loginfo('node is running..')

    rospy.Subscriber('joint_states', JointState, joint_states_callback)

    while not _ok:
        rospy.sleep(.1)
