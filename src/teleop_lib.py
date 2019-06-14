#!/usr/bin/python


class JoyTools:
    press = 1

    class application:
        WIRELESS = 0
        WIRED = 1
        LOGITECH_F310 = 1
        LOGITECH_F710 = 2

    class buttons:
        A = 0
        B = 0
        X = 0
        Y = 0
        LB = 0
        RB = 0
        back = 0
        start = 0
        power = 0
        RT = 0
        LT = 0

        class stick:
            class right:
                x = 0
                y = 0
                click = 0

            class left:
                x = 0
                y = 0
                click = 0

        class cross:
            x = 0
            y = 0

    def __init__(self, application, debug=False):
        self.select_application = application
        self.debug = debug
        self.locked = True
        self.RT_PRESS = False
        self.msg = None
        if self.select_application == self.application.LOGITECH_F710:
            self.is_RT_press = lambda x: True if x > 0 else False
        else:
            self.is_RT_press = lambda x: True if x <= 0 else False

    def joy_map(self, axes, buttons):
        """
            Params:
                application:
                    - app.WIRELESS
                        - Microsoft Xbox 360 Wireless Controller for Linux
                    - app.WIRED
                        - Microsoft Xbox 360 Wired Controller for Linux
                        - Incl. Logitech F310 joy
                    - app.LOGITECH_F710
                        - Logitech Wireless Gamepad F710 (DirectInput Mode)
        """
        if self.select_application == self.application.WIRELESS:
            self.buttons.A = buttons[0]
            self.buttons.B = buttons[1]
            self.buttons.X = buttons[2]
            self.buttons.Y = buttons[3]
            self.buttons.LB = buttons[4]
            self.buttons.RB = buttons[5]
            self.buttons.back = buttons[6]
            self.buttons.start = buttons[7]
            self.buttons.power = buttons[8]
            self.buttons.stick.left.click = buttons[9]
            self.buttons.stick.right.click = buttons[10]
            self.buttons.stick.left.x = axes[0]
            self.buttons.stick.left.y = axes[1]
            self.buttons.stick.right.x = axes[2]
            self.buttons.stick.right.y = axes[3]
            self.buttons.RT = axes[4]
            self.buttons.LT = axes[5]
            self.buttons.cross.x = axes[6]
            self.buttons.cross.y = axes[7]

        elif self.select_application == self.application.WIRED:
            self.buttons.A = buttons[0]
            self.buttons.B = buttons[1]
            self.buttons.X = buttons[2]
            self.buttons.Y = buttons[3]
            self.buttons.LB = buttons[4]
            self.buttons.RB = buttons[5]
            self.buttons.back = buttons[6]
            self.buttons.start = buttons[7]
            self.buttons.power = buttons[8]
            self.buttons.stick.left.click = buttons[9]
            self.buttons.stick.right.click = buttons[10]
            self.buttons.stick.left.x = axes[0]
            self.buttons.stick.left.y = axes[1]
            self.buttons.LT = axes[2]
            self.buttons.stick.right.x = axes[3]
            self.buttons.stick.right.y = axes[4]
            self.buttons.RT = axes[5]
            self.buttons.cross.x = axes[6]
            self.buttons.cross.y = axes[7]

        elif self.select_application == self.application.LOGITECH_F710:
            self.buttons.X = buttons[0]
            self.buttons.A = buttons[1]
            self.buttons.B = buttons[2]
            self.buttons.Y = buttons[3]
            self.buttons.LB = buttons[4]
            self.buttons.RB = buttons[5]
            self.buttons.LT = buttons[6]
            self.buttons.RT = buttons[7]
            self.buttons.back = buttons[8]
            self.buttons.start = buttons[9]
            self.buttons.stick.left.click = buttons[10]
            self.buttons.stick.right.click = buttons[11]
            self.buttons.stick.left.x = axes[0]
            self.buttons.stick.left.y = axes[1]
            self.buttons.stick.right.x = axes[2]
            self.buttons.stick.right.y = axes[3]
            self.buttons.cross.x = axes[4]
            self.buttons.cross.y = axes[5]

    def callback(self, msg):
        self.msg = msg
        axes = list(msg.axes)
        buttons = list(msg.buttons)
        self.joy_map(axes, buttons)
        self.RT_PRESS = self.is_RT_press(self.buttons.RT)
        if(self.debug):
            self.print_debug()

    def print_debug(self):
        status = ['not press', 'pressed']
        print("A:       " + status[self.buttons.A])
        print("B:       " + status[self.buttons.B])
        print("X:       " + status[self.buttons.X])
        print("Y:       " + status[self.buttons.Y])
        print("LB:      " + status[self.buttons.LB])
        print("RB:      " + status[self.buttons.RB])
        print("LT:      " + str(self.buttons.LT))
        print("RT:      " + str(self.buttons.RT))
        if self.select_application == self.application.LOGITECH_F710:
            print("LT:      " + status[self.buttons.LT])
            print("RT:      " + status[self.buttons.RT])
        else:
            print("LT:      " + str(self.buttons.LT))
            print("RT:      " + str(self.buttons.RT))
        print("CROSS_X: " + str(self.buttons.cross.x))
        print("CROSS_Y: " + str(self.buttons.cross.y))
        print("LEFT_X:  " + str(self.buttons.stick.left.x))
        print("LEFT_Y:  " + str(self.buttons.stick.left.y))
        print("LEFT_C:  " + status[self.buttons.stick.left.click])
        print("RIGHT_X: " + str(self.buttons.stick.right.x))
        print("RIGHT_Y: " + str(self.buttons.stick.right.y))
        print("RIGHT_C: " + status[self.buttons.stick.right.click])
        if self.select_application != self.application.LOGITECH_F710:
            print("POWER:   " + status[self.buttons.power])
        print("BACK:    " + status[self.buttons.back])
        print("START:   " + status[self.buttons.start])


class Convert:
    def __init__(self):
        import constant
        self.X_MAX = constant.X_MAX
        self.Y_MAX = constant.Y_MAX
        self.Z_MAX = constant.Z_MAX
        self.YAW_MAX = constant.YAW_MAX

    def to_x(self, inp):
        if abs(inp) < 1e-3:
            return None
        sign = 1
        return sign*inp*self.X_MAX

    def to_y(self, inp):
        if abs(inp) < 1e-3:
            return None
        sign = 1
        return sign*inp*self.Y_MAX

    def to_z(self, inp, default_z=0):
        sign = -1
        if abs(inp) < 1e-3 and default_z == 0:
            return None
        elif abs(inp) < 1e-3 and default_z != 0:
            return default_z
        return sign*(inp*self.Z_MAX)

    def to_yaw(self, inp):
        if abs(inp) < 1e-3:
            return None
        sign = 1
        return sign*inp*self.YAW_MAX
