#!/usr/bin/python
import rospy
import os
from sensor_msgs.msg import Joy
from joy_lib import JoyTools


def func():
  if(joy.buttons.RT > 0):
    if(joy.buttons.A == joy.press):
      print('driving')
    if(joy.buttons.Y == joy.press):
      print('upping')
    print('X',joy.buttons.stick.left.x)
    print('Y',joy.buttons.stick.left.y)
    print('Yaw',joy.buttons.stick.right.x)
  else:
    print('Press RT to control')



joy = JoyTools(JoyTools.application.LOGITECH_F710,debug=False)


if __name__ == '__main__':
    rospy.init_node('teleop_joy', anonymous=False)
    rospy.Subscriber("joy", Joy, callback=joy.callback,callback_args=func,queue_size=1)
    # os.system('clear')
    print('Press RT to control')
    rospy.spin()