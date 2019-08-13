import numpy as np
import math

class Kinematics():
    def __init__(self,l0=0.061,l1=0.046,l2=0.060,l3=0.085):
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
            beta = 0 + math.atan(z/xp)
            gamma = 0
            #print("Warning!")
        elif(d<=self.l3-self.l2):
            d = self.l3-self.l2
            beta = 0 + math.atan(z/xp)
            gamma = -math.pi/2
            #print("Warning!")
        else:
            #print("z=",-z)
            beta = self.AlKashi(self.l2,d,self.l3) + math.atan(z/xp)
            gamma = -math.pi + self.AlKashi(self.l2,self.l3,d)

        if(alpha is not None and beta is not None and gamma is not None):
            #a = math.degrees(alpha)
            #b = math.degrees(beta)
            #c = math.degrees(gamma)
            return [alpha,-beta,gamma]
        else:
            print("IK ERROR: Joint position is None!")
            return None

    def FK(self, joint_pos, leg_idx):
        alpha = joint_pos[0]
        beta = -joint_pos[1]
        gamma = joint_pos[2]

        r = self.l1+self.l2*math.cos(beta)+self.l3*math.cos(beta+gamma)

        x = r*math.cos(alpha)
        y = r*math.sin(alpha)
        z = self.l2*math.sin(beta)+self.l3*math.sin(beta+gamma)

        return self.inverse_coordinate_transformation([x,y,z], leg_idx)

    def coordinate_transformation(self, body_frame, leg_idx):
    #body_frame = [X,Y,Z]
        idx = 3-leg_idx
        X = body_frame[0]
        Y = body_frame[1]
        #Z = body_frame[2]

        x = math.cos(idx*math.pi/2-math.pi/4)*X - math.sin(idx*math.pi/2-math.pi/4)*Y + self.l0
        y = math.sin(idx*math.pi/2-math.pi/4)*X + math.cos(idx*math.pi/2-math.pi/4)*Y
        
        return x,y

    def inverse_coordinate_transformation(self, leg_frame, leg_idx):
        idx = 3-leg_idx
        X = leg_frame[0] + self.l0
        Y = leg_frame[1]
        Z = leg_frame[2]

        x = -math.cos(-idx*math.pi/2+math.pi/4)*X + math.sin(-idx*math.pi/2+math.pi/4)*Y
        y = -math.sin(-idx*math.pi/2+math.pi/4)*X - math.cos(-idx*math.pi/2+math.pi/4)*Y
        z = Z
        return [x,y,z]

    def AlKashi(self, a, b, c):
        #print("a,b,c = ",a,b,c)
        #print(((a*a)+(b*b)-(c*c))/(2*a*b))
        return abs(math.acos(((a*a)+(b*b)-(c*c))/(2*a*b)))
