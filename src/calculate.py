class Convert:
    def __init__(self):
        self.X_MAX = 2.0
        self.Y_MAX = 2.0
        self.Z_MAX = 2.4
        self.YAW_MAX = 0.5

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
