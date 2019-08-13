import numpy as np
import Kinematics.Kinematics_firmware as kin
from Vrep.vrep_simulation import obstacle, environment
import time

env = environment()
env.start_simulation()
env.update()
env.robot_go_initial()
env.update()

dx = 50.0
dy = 50.0
alt = 15.0
radius = 153.0
x_ = np.cos(np.pi/4) * radius
y_ = np.cos(np.pi/4) * radius
kin_solver = kin.Kinematics()

initial_joint_position = np.array([0.0, -29.76, -123.87, 
                              0.0, -29.76, -123.87, 
                              0.0, -29.76, -123.87, 
                              0.0, -29.76, -123.87]) * np.pi/180
joint_position = list(initial_joint_position)

for step in np.arange(1.0, 1.2, 0.2):
    for rise in np.arange(1.0, 6.0, 0.8):
        print("step=", step)
        print("rise=", rise)
        print("")
        print("")
        x = x_ + step * dx
        y = y_ + step * dy
        z = rise * alt
        
        L = kin_solver.IK([x, y, z], 0)
        joint_position[0] = L[0]
        joint_position[1] = L[1]
        joint_position[2] = L[2]
        env.update()
        env.set_joint_position(env.clientID, env.Joint_Handles, joint_position, env.curr_joint_position)
        time.sleep(1)
