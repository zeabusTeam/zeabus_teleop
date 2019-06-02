class Convert:
    def __init__(self):
        self.X_MAX = 1.0
        self.Y_MAX = 1.0
        self.Z_MAX = 1.0
        self.YAW_MAX = 1.0

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

    def to_z(self, inp):
        if abs(inp) < 1e-3:
            return None
        sign = 1
        return sign*inp*self.Z_MAX

    def to_yaw(self, inp):
        if abs(inp) < 1e-3:
            return None
        sign = 1
        return sign*inp*self.YAW_MAX
