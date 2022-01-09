import numpy as np


class Wall():
    def __init__(self, width, thick, center=(0, 0)):
        self.w = width
        self.t = thick
        self.cx, self.cy = center
        self.x_min = self.cx - self.w
        self.x_max = self.cx + self.w
        self.y_min = self.cy - self.t
        self.y_max = self.cy + self.t

        if self.w > self.t:
            self.p1 = (self.x_min, self.cy)
            self.p2 = (self.x_max, self.cy)
            self.orientation = 1  # Vertical
        else:
            self.p1 = (self.cx, self.y_max)
            self.p2 = (self.cx, self.y_min)
            self.orientation = 0  # Horizontal

    def potential_field(self, x, y):
        t_num = (self.p1[0] - x)*(self.p2[0] - self.p1[0]) + (self.p1[1] - y)*(self.p2[1] - self.p1[1])
        t_den = np.power(self.p2[0] - self.p1[0], 2) + np.power(self.p2[1] - self.p1[1], 2)
        t = -t_num/t_den

        if 0 <= t and t <= 1:
            d_num = (self.p2[0] - self.p1[0])*(self.p1[1] - y) - (self.p2[1] - self.p1[1])*(self.p1[0] - x)
            d = np.abs(d_num)/np.sqrt(t_den)
            V = field_1d(d)
            if self.orientation == 1:  # Veritcal self
                dx = 0
                dy = V * 1 if y - self.p1[1] >= 0 else V * -1
            elif self.orientation == 0:  # Horizontal self
                dx = V * 1 if x - self.p1[0] >= 0 else V * -1
                dy = 0
        else:
            d1 = np.power(self.p2[0] - x, 2) + np.power(self.p2[1] - y, 2)
            d2 = np.power(self.p1[0] - x, 2) + np.power(self.p1[1] - y, 2)
            if d1 < d2:
                d = np.sqrt(d1)
                V = field_1d(d)
                sign = 1 if x - self.p2[0] >= 0 else -1
                dx = V * np.abs((self.p2[0] - x)/d) * sign
                
                sign = 1 if y - self.p2[1] >= 0 else -1
                dy = V * np.abs((self.p2[1] - y)/d) * sign
            else:
                d = np.sqrt(d2)
                V = field_1d(d)
                sign = 1 if x - self.p1[0] >= 0 else -1
                dx = V * np.abs((self.p1[0] - x)/d) * sign

                sign = 1 if y - self.p1[1] >= 0 else -1
                dy = V * np.abs((self.p1[1] - y)/d) * sign

        return dx, dy

def field_1d(x):
    if x < 0.4:
        V = 5
    else:
        V = 0
    return V

def room_potential(x, y):

    wall1 = Wall(0.5, 2, (3, 0))
    wall2 = Wall(1, 0.5, (1, 2))
    walls = [wall1, wall2]

    dx = 0
    dy = 0

    for w in walls:
        _dx, _dy = w.potential_field(x, y)
        dx += _dx
        dy += _dy

    return dx, dy