import numpy as np
import math

class Kinematics():
    def __init__(self,l0=61.5,l1=46,l2=59,l3=85):
        self.l0 = l0
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3

    def IK(self, target_ref_body, leg_idx):
        x,y = self.coordinate_transformation(target_ref_body, leg_idx)
        z = target_ref_body[2]
        alpha = math.atan(y/x)

        xp = math.sqrt(x**2+y**2)-self.l1
        if(xp<0):
            xp=1e-5

        d = math.sqrt(xp**2+z**2)
        if(d>=self.l2+self.l3):
            d = self.l2+self.l3
            beta = 0 - math.atan(-z/xp)
            gamma = 0
            #print("Warning!")
        elif(d<=self.l3-self.l2):
            d = self.l3-self.l2
            beta = math.pi - math.atan(-z/xp)
            gamma = math.pi/2
            #print("Warning!")
        else:
            beta = self.AlKashi(self.l2,d,self.l3) - math.atan(-z/xp)
            gamma = math.pi - self.AlKashi(self.l2,self.l3,d)

        if(alpha is not None and beta is not None and gamma is not None):
            #a = math.degrees(alpha)
            #b = math.degrees(beta)
            #c = math.degrees(gamma)
            return [-alpha,-beta,-gamma]
        else:
            print("IK ERROR: Joint position is None!")
            return None

    def coordinate_transformation(self, body_frame, leg_idx):
    #body_frame = [X,Y,Z]
        idx = leg_idx
        X = body_frame[0]
        Y = body_frame[1]
        #Z = body_frame[2]

        x = math.cos(idx*math.pi/2-math.pi/4)*X - math.sin(idx*math.pi/2-math.pi/4)*Y - self.l0
        y = math.sin(idx*math.pi/2-math.pi/4)*X + math.cos(idx*math.pi/2-math.pi/4)*Y
        
        return x,y

    def AlKashi(self, a, b, c):
        return abs(math.acos(((a*a)+(b*b)-(c*c))/(2*a*b)))
