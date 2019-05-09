#!/usr/bin/python
import rospy
import os
from sensor_msgs.msg import Joy
from joy_lib import JoyTools

joy = JoyTools(JoyTools.application.LOGITECH_F310)


if __name__ == '__main__':
    rospy.init_node('teleop_joy', anonymous=False)
    rospy.Subscriber("joy", Joy, joy.callback)
    rospy.spin()