#!/usr/bin/python
import rospy
import os
from sensor_msgs.msg import Joy
from zeabus_utility.msg import ControlCommand
from zeabus_utility.srv import SendControlCommand
from joy_lib import JoyTools

call = rospy.ServiceProxy('/control/thruster', SendControlCommand)


def message(header, x=None, y=None, z=None, yaw=None):
    MAX = 1
    msg = ControlCommand()
    msg.header = header
    target = [0.0] * 6		# [x, y, z, roll, pitch, yaw]
    mask = [False] * 6		# [x, y, z, roll, pitch, yaw]
    if x is not None:
        target[0] = float(x*MAX)
        mask[0] = True
    if y is not None:
        target[1] = float(y*MAX)
        mask[1] = True
    if z is not None:
        target[2] = float(z*MAX)
        mask[2] = True
    if yaw is not None:
        target[5] = float(yaw*MAX)
        mask[5] = True
    msg.target = target
    msg.mask = mask
    print(msg)
    return msg


def func(header, locked):
    try:
        if(joy.RT_PRESS):
            z = 0.0
            if(joy.buttons.A == joy.press and joy.buttons.Y != joy.press):
                print('driving')
                z = -1.0
            elif(joy.buttons.Y == joy.press and joy.buttons.A != joy.press):
                print('upping')
                z = 1.0
            print('X', joy.buttons.stick.left.x)
            print('Y', joy.buttons.stick.left.y)
            print('Yaw', joy.buttons.stick.right.x)
            call(message(header=header,
                        x=joy.buttons.stick.left.x,
                        y=joy.buttons.stick.left.y,
                        z=z,
                        yaw=joy.buttons.stick.right.x))
            return False
        else:
            if locked == False:
                call(message(header=header))
            print('Press RT to control')
            return True
    except:
        print('Code failed (Check topic or thruster node)')
        return True


joy = JoyTools(JoyTools.application.LOGITECH_F310, debug=False)


if __name__ == '__main__':
    rospy.init_node('teleop_joy', anonymous=False)
    rospy.Subscriber("joy", Joy, callback=joy.callback,
                     callback_args=func, queue_size=3)
    print('Press RT to control')
    rospy.spin()
