import numpy as np
import Kinematics.Kinematics_firmware as kin

class point:
    def __init__(self, x, y, yp):
        self.x = x
        self.y = y
        self.yp = yp
        
class polynom:
    def __init__(self, a, b, c, d):
        self.a = a
        self.b = b
        self.c = c
        self.d = d

class cubic:
    def __init__(self):
        self.nbpoints = 0
        self.points = list()
        self.polynoms = list()
        
    def get_xmax(self):
        if self.nbpoints == 0:
            return 0.0
        else:
            return self.points[self.nbpoints-1].x
            
    def cal_polynom(self, pre_idx, curr_idx):
        if pre_idx >= self.nbpoints or pre_idx < 0:
            print("[ERROR] Previous Point Idx out of Range")
        if curr_idx >= self.nbpoints or curr_idx < 0:
            print("[ERROR] Current Point Idx out of Range")
        pre_point = self.points[pre_idx]
        curr_point = self.points[curr_idx]
        a = 2*pre_point.y + pre_point.yp - 2*curr_point.y + curr_point.yp
        b = -3*pre_point.y - 2*pre_point.yp + 3*curr_point.y - curr_point.yp
        c = pre_point.yp
        d = pre_point.y
        return a, b, c, d
        
    def add_point(self, x, y, yp):
        self.points.append(point(x, y, yp))
        self.nbpoints += 1
        if self.nbpoints > 1:
            a, b, c, d = self.cal_polynom(self.nbpoints-2, self.nbpoints-1)
            self.polynoms.append(polynom(a, b, c, d))
        
    def change_point(self, point_idx, y):
        if point_idx >= self.nbpoints:
            print("[ERROR] Point Idx out of Range")
            return
        else:
            x = self.points[point_idx].x
            yp = self.points[point_idx].yp
            self.points[point_idx] = point(x, y, yp)
            
        if point_idx < self.nbpoints - 1:
            a, b, c, d = self.cal_polynom(point_idx, point_idx+1)
            self.polynoms[point_idx] = polynom(a, b, c, d)
            
        if point_idx > 0:
            a, b, c, d = self.cal_polynom(point_idx-1, point_idx)
            self.polynoms[point_idx-1] = polynom(a, b, c, d)
        
    def get(self, x):
        for idx in range(self.nbpoints):
            if self.points[idx].x >= x:
                break
                
        if idx == 0 or idx == self.nbpoints:
            return 0.0
            
        A = self.points[idx-1]
        B = self.points[idx]
        f = self.polynoms[idx-1]
        x = (x-A.x)/(B.x-A.x)
        
        tx = x
        result = f.d
        result += f.c * tx
        tx *= x
        result += f.b * tx
        tx *= x
        result += f.a * tx
        
        return result
        
    def get_mod(self, x):
        max_x = self.get_xmax()
        if x < 0.0 or x > max_x:
            x = x % max_x
        return self.get(x)
             
class motion:
    def __init__(self):
        self.r = 153.0 # robot size
        self.h = -55.0 # robot height
        self.alt = 15.0 # robot step height
        self.rise_left = cubic()
        self.rise_right = cubic()
        self.step_left = cubic()
        self.step_right = cubic()
        self.phasesA = [0.0, 0.5, 1.0, 0.5]
        self.phasesB = [0.0, 0.5, 0.75, 0.25]
        self.gait = 1
        self.front_h_left = 0
        self.front_h_right = 0
        self.extra_r = 0
        self.extra_h = 0
        self.freq = 2.0
        self.dx = 0
        self.dy = 0
        self.pos = [[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0]]
        self.kin_solver = kin.Kinematics()
        
    def motion_tick(self, t):
        l1 = list()
        l2 = list()
        l3 = list()
        angle = np.arctan2(self.dy, self.dx) - np.pi/4 # angle to the first leg
        for leg_idx in range(4):
            leg_phase = t + self.phasesA[leg_idx] * self.gait + self.phasesB[leg_idx] * (1-self.gait)
            local_angle = angle + np.pi*leg_idx/2
            local_angle = local_angle - np.floor(local_angle/(2*np.pi))*2*np.pi
            is_left = local_angle < np.pi
            is_back = local_angle >= np.pi/2 and local_angle < np.pi*3/2
            if is_left:
                stepping = self.step_left.get_mod(leg_phase)
                rising = self.rise_left.get_mod(leg_phase)
            else:
                stepping = self.step_right.get_mod(leg_phase)
                rising = self.rise_right.get_mod(leg_phase)
            radius = self.r + self.extra_r
            
            x = np.cos(np.pi/4) * radius * (1 if leg_idx == 0 or leg_idx == 1 else -1)
            y = np.cos(np.pi/4) * radius * (1 if leg_idx == 0 or leg_idx == 3 else -1)

            x += stepping * self.dx
            y += stepping * self.dy
            
            #x, y = self.kin_solver.coordinate_transformation([X, Y], leg_idx)
            z = self.h - self.extra_h + rising * self.alt
            if is_back:
                #print("motion::leg_idx ",leg_idx)
                if is_left:
                    z -= self.front_h_left
                else:
                    z -= self.front_h_right
                
            self.pos[leg_idx] = [x, y, z]
            # compute IK
            L = self.kin_solver.IK([x, y, z], leg_idx)

            if L is None:
                print("[ERROR] Motion Error")
                return None
            else:
                l1.append(L[0])
                l2.append(L[1])
                l3.append(L[2])
                
        return l1, l2, l3
    
    def motion_set_f(self, f):
        self.freq = f
        
    def motion_set_h(self, h):
        self.extra_h = h
        
    def motion_set_r(self, r):
        self.extra_r = r
        
    def motion_set_front_h(self, h_left, h_right):
        self.front_h_left = h_left
        self.front_h_right = h_right
        
    def motion_set_x_speed(self, x_speed):
        self.dx = 1.2*x_speed/(2.0*self.freq)
        
    def motion_set_y_speed(self, y_speed):
        self.dy = 1.2*y_speed/(2.0*self.freq)

    def get_position(self):
        return self.pos
