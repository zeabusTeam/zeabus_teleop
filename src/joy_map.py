#!/usr/bin/python
import rospy
import os
from sensor_msgs.msg import Joy
from zeabus_utility.msg import ControlCommand, TelemetryStruct
from zeabus_utility.srv import SendControlCommand, Telemetry
from teleop_lib import JoyTools, Convert
from time import sleep


convert = Convert()


def calculate_voltage(data):
    sum_voltage = 0
    for i in data:
        sum_voltage += i.voltage
    return sum_voltage/len(data)

def break_message(header, seq):
    msg = ControlCommand()
    msg.header = header
    msg.header.seq = seq
    msg.target = [0.0] * 6
    msg.mask = [True] * 6
    return msg


def message(header, seq=0, x=None, y=None, z=None, yaw=None, reset=False):
    if(x is None and y is None and z is None and yaw is None and not reset):
        return []
    msg = ControlCommand()
    msg.header = header
    msg.header.seq = seq
    target = [0.0] * 6		# [x, y, z, roll, pitch, yaw]
    mask = [False] * 6		# [x, y, z, roll, pitch, yaw]
    if x is not None:
        target[0] = float(x)
        mask[0] = True
    if y is not None:
        target[1] = float(y)
        mask[1] = True
    if z is not None:
        target[2] = float(z)
        mask[2] = True
    if yaw is not None:
        target[5] = float(yaw)
        mask[5] = True
    msg.target = target
    msg.mask = mask
    return msg


def run():
    pub = rospy.Publisher('/control/thruster', ControlCommand)
    bat_check_call = rospy.ServiceProxy('/hardware/thruster_feedback', Telemetry)
    # call = rospy.ServiceProxy('/control/thruster', SendControlCommand)
    locked = True
    not_break_rule = True
    last_battery = None
    default_z = 0
    i = 1
    while not rospy.is_shutdown():
        os.system('clear')
        print('default_z: ',default_z)
        print('break_rule', not_break_rule)
        print('Last check battery', last_battery)
        if(joy.RT_PRESS and not_break_rule):
            msg = message(header=joy.msg.header,
                          seq=i,
                          x=convert.to_x(joy.buttons.stick.left.y),
                          y=convert.to_y(joy.buttons.stick.left.x),
                          z=convert.to_z(joy.buttons.A, default_z),
                          yaw=convert.to_yaw(joy.buttons.stick.right.x))
            if msg != []:
                pub.publish(msg)
                print(msg)
                i += 1
            else:
                print('Waiting')
            locked = False
        elif(joy.buttons.power == joy.press):
            print('checking battery')
            response = bat_check_call()
            last_battery = calculate_voltage(response.telemetry)
        elif(joy.buttons.LB == joy.press and joy.buttons.start == joy.press):
            not_break_rule = True
        elif(joy.buttons.RB == joy.press and joy.buttons.start == joy.press):
            not_break_rule = False
        elif(not_break_rule == False):
            msg = break_message(header=joy.msg.header, seq=i)
            pub.publish(msg)
            print(msg)
            i += 1
        elif(joy.buttons.LB == joy.press and joy.buttons.back == joy.press):
            break
        elif(joy.buttons.LB == joy.press and joy.buttons.Y == joy.press):
            default_z = 0
        elif(joy.buttons.RB == joy.press and joy.buttons.Y == joy.press):
            default_z = convert.DEFAULT_Z
        else:
            print('Press RT to control')
            print('Press RB+Y to set default_z = ' + str(convert.DEFAULT_Z))
            print('Press LB+Y to set default_z = 0')
            print('Press RB+START to disable force stop AUV')
            print('Press LB+START to enable force stop AUV')
            print('Press power to check battery (F310 only)')
            if default_z != 0:
                pub.publish(message(header=joy.msg.header, seq=i,z=default_z))
                i+=1
            elif locked == False:
                pub.publish(message(header=joy.msg.header,seq=i,reset=True))
                i+=1
            locked = True
        sleep(0.1)
    print('End loop')


joy = JoyTools(JoyTools.application.LOGITECH_F310, debug=False)


if __name__ == '__main__':
    rospy.init_node('teleop_joy', anonymous=False)
    rospy.Subscriber("joy", Joy, callback=joy.callback, queue_size=3)
    run()
    rospy.spin()
