try:
    from . import vrep
except:
    print ('--------------------------------------------------------------')
    print ('"vrep.py" could not be imported. This means very probably that')
    print ('either "vrep.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "vrep.py"')
    print ('--------------------------------------------------------------')
    print ('')
import numpy as np
import math
import time
import os
from Motion.motion import motion
from TF.tf_2d import tf_2d
from TF.tf_3d import tf_3d
###########################################PREDEFINED VARIABLES#########################################################################################
              
class obstacle:
    position = [0,0]
    height = 0
    length = 0
    width = 0
    orientation = 0
    def __init__(self,position,height,length=None,width=None,orientation=None):
        self.position = position
        self.height = height
        self.length = length
        self.width = width
        self.orientation = orientation


class environment:
    def __init__(self):
        self.tf_2d = tf_2d()
        self.tf_3d = tf_3d()
        self.clientID = None
        self.start_client()
        Body = 'body_1_respondable'
        Joints = ['double_u_2_parent', 'side_to_side_3_parent', 'arm_leg_4_parent', 
                  'double_u_5_parent', 'side_to_side_6_parent', 'arm_leg_7_parent', 
                  'double_u_8_parent', 'side_to_side_9_parent', 'arm_leg_10_parent', 
                  'double_u_11_parent', 'side_to_side_12_parent', 'arm_leg_13_parent']
                  
        self.Body_Handle = self.get_body_handle(self.clientID, Body) # get body handle
        self.Joint_Handles = self.get_joint_handle(self.clientID, Joints) # get joint handles
        if self.Joint_Handles is None:
            print ('[ERROR] Handle Error')
            return
        
        self.set_joint_position_streaming(self.clientID, self.Joint_Handles)
        self.set_joint_force_streaming(self.clientID, self.Joint_Handles)
        self.set_body_position_streaming(self.clientID, self.Body_Handle)
        self.set_body_orientation_streaming(self.clientID, self.Body_Handle)
        self.set_body_velocity_streaming(self.clientID, self.Body_Handle)

        self.curr_robot_position = None
        self.prev_robot_position = None
        self.curr_robot_angle = None
        self.curr_robot_orientation = None
        self.curr_joint_position = None
        self.action = 0
        self.initial_joint_position = np.array([0.0, -29.76, -123.87, 
                              0.0, -29.76, -123.87, 
                              0.0, -29.76, -123.87, 
                              0.0, -29.76, -123.87]) * math.pi/180
        self.initial_joint_position = list(self.initial_joint_position)
        self.incline_threshold = 2.8
        self.is_incline = False
        self.states = None
        self.distance = 0.0
        self.prev_distance = 0.0
        self.distance_reward = 0.0
        self.local_goal_direction = 0.0
        # public variables to store goal position and obstable positions and heights
        self.goal_position = np.array([-1,-1])
        self.support = 0.5
        self.speed = 200.0
        self.mot = motion()
        self.setup_motion()
        self.obstacle = obstacle(position=[-10,-10],height=0.0)

        self.action4_single_leg = []
        step = [-0.5,0.0,0.5]
        rise = [1.0,4.0,7.0]
        front_h = [0.0,45.0,90.0]
        for i in range(len(step)):
            for j in range(len(rise)):
                for k in range(len(front_h)):
                    self.action4_single_leg.append([step[i],rise[j],front_h[k]])

    def start_client(self):
        vrep.simxFinish(-1) # just in case, close all opened connections
        self.clientID = vrep.simxStart('127.0.0.1', 4242, True, True, 2000, 5) # connect to V-REP
        if self.clientID!=-1:
            print ('[INFO] Connected to remote API server') 
        else:
            print ('[ERROR] Connection failed')
            return
            
    def stop_client(self):
        if self.clientID is not None:
            vrep.simxFinish(self.clientID)
        else:
            print('[ERROR] Clinet not start')

    def start_simulation(self):
        if self.clientID is not None:
            vrep.simxStartSimulation(self.clientID, vrep.simx_opmode_blocking) # start the simulation
        else:
            print('[ERROR] Clinet not start')
            
    def stop_simulation(self):
        if self.clientID is not None:
            vrep.simxStopSimulation(self.clientID, vrep.simx_opmode_blocking)
        else:
            print('[ERROR] Clinet not start')
            
    def setup_motion(self):
        self.mot.step_left.add_point(0.0, 0.5, -1.0/self.support)
        self.mot.step_left.add_point(self.support, -0.5, -1.0/self.support)
        self.mot.step_left.add_point(self.support+(1.0-self.support)/2.0, 0.0, 1.0)
        self.mot.step_left.add_point(1.0, 0.5, -1.0/self.support)
        
        self.mot.rise_left.add_point(0.0, 0.0, 0.0)
        self.mot.rise_left.add_point(self.support, 0.0, 0.0)
        self.mot.rise_left.add_point(self.support+(1.0-self.support)/2.0, 1.0, 0.0)
        self.mot.rise_left.add_point(1.0, 0.0, 0.0)

        self.mot.step_right.add_point(0.0, 0.5, -1.0/self.support)
        self.mot.step_right.add_point(self.support, -0.5, -1.0/self.support)
        self.mot.step_right.add_point(self.support+(1.0-self.support)/2.0, 0.0, 1.0)
        self.mot.step_right.add_point(1.0, 0.5, -1.0/self.support)
        
        self.mot.rise_right.add_point(0.0, 0.0, 0.0)
        self.mot.rise_right.add_point(self.support, 0.0, 0.0)
        self.mot.rise_right.add_point(self.support+(1.0-self.support)/2.0, 1.0, 0.0)
        self.mot.rise_right.add_point(1.0, 0.0, 0.0)
    
    def robot_go_initial(self):
        self.curr_joint_position = None
        while self.curr_joint_position is None:
            self.curr_joint_position = self.get_joint_position_streaming(self.clientID, self.Joint_Handles)
        self.set_joint_position(self.clientID, self.Joint_Handles, self.initial_joint_position, self.curr_joint_position)
        
    
        while np.linalg.norm(np.asarray(self.initial_joint_position) - np.asarray(self.curr_joint_position)) > 0.11:
            self.curr_joint_position = self.get_joint_position_streaming(self.clientID, self.Joint_Handles)

        self.action = 0
        self.update()
        self.prev_joint_position = self.curr_joint_position
        self.prev_distance = self.distance
        self.is_incline = False

    def update(self):
        self.prev_joint_position = self.curr_joint_position
        self.curr_joint_position = self.get_joint_position_streaming(self.clientID, self.Joint_Handles)
        self.curr_robot_position = self.get_body_position_streaming(self.clientID, self.Body_Handle)
        self.curr_robot_angle = self.get_body_orientation_streaming(self.clientID, self.Body_Handle)
        self.prev_distance = self.distance
        self.tf_3d.set_tf(self.curr_robot_position[0], self.curr_robot_position[1], self.curr_robot_position[2], self.curr_robot_angle[0], self.curr_robot_angle[1], self.curr_robot_angle[2])
        robot_orientation = self.tf_3d.trans_matrix[0:3,0:3]*np.matrix([1.0,0.0,0.0]).T
        robot_incline_angle = np.arccos(np.dot(np.array([robot_orientation[0,0], robot_orientation[1,0], robot_orientation[2,0]]), np.array([0.0, 0.0, 1.0])) / (np.linalg.norm(np.array([robot_orientation[0,0], robot_orientation[1,0], robot_orientation[2,0]])) * np.linalg.norm(np.array([0, 0, 1]))))
        if robot_incline_angle > self.incline_threshold:
            self.is_incline = True
        robot_direction_3d = self.tf_3d.trans_matrix[0:3,0:3]*np.matrix([0.0,1.0,0.0]).T
        robot_direction = np.arctan2(robot_direction_3d[1,0], robot_direction_3d[0,0])
        self.tf_2d.set_tf(self.curr_robot_position[0], self.curr_robot_position[1], robot_direction)
        local_goal_y, local_goal_x = self.tf_2d.global_to_local(self.goal_position[0], self.goal_position[1])
        self.local_goal_direction = np.arctan2(local_goal_y, local_goal_x)
        #print("curr_robot_position:", self.curr_robot_position)
        #print("robot_direction:", robot_direction)
        #print("local_goal_direction:", local_goal_direction)
        #print("x:", local_goal_x)
        #print("y:", local_goal_y)
        self.distance = np.linalg.norm([local_goal_x, local_goal_y])
        self.states = self.get_states()

    def get_body_handle(self, clientID, body):
        res, body_handle = vrep.simxGetObjectHandle(clientID, body, vrep.simx_opmode_blocking)
        if res != vrep.simx_return_ok:
            print ('[ERROR] Handle Error for Body %s' % (body))
            return None
        else:
            print ('[INFO] Handle %d is set to Body %s' % (body_handle, body))
            return body_handle

    def set_body_position_streaming(self, clientID, body_handle):
        vrep.simxGetObjectPosition(clientID, body_handle, -1, vrep.simx_opmode_streaming)
            
    def get_body_position_streaming(self, clientID, body_handle):
        res, body_position = vrep.simxGetObjectPosition(clientID, body_handle, -1, vrep.simx_opmode_buffer)
        if res != vrep.simx_return_ok:
            print ('[ERROR] Get Position for Body %d failed with Error Code %d' % (body_handle, res))
            return None
        else:
            return body_position

    def set_body_orientation_streaming(self, clientID, body_handle):
        vrep.simxGetObjectOrientation(clientID, body_handle, -1, vrep.simx_opmode_streaming)

    def get_body_orientation_streaming(self, clientID, body_handle):
        res, body_orientation = vrep.simxGetObjectOrientation(clientID, body_handle, -1, vrep.simx_opmode_buffer)
        if res != vrep.simx_return_ok:
            #print ('[ERROR] Get Orientation for Body %d failed with Error Code %d' % (body_handle, res))
            return None
        else:
            return body_orientation
            
    def set_body_velocity_streaming(self, clientID, body_handle):
        vrep.simxGetObjectVelocity(clientID, body_handle, vrep.simx_opmode_streaming)
            
    def get_body_velocity_streaming(self, clientID, body_handle):
        res, body_velocity, _ = vrep.simxGetObjectVelocity(clientID, body_handle, vrep.simx_opmode_buffer)
        if res != vrep.simx_return_ok:
            print ('[ERROR] Get Position for Body %d failed with Error Code %d' % (body_handle, res))
            return None
        else:
            return body_velocity

    def get_joint_handle(self, clientID, Joints):
        Joint_Handles = list()
        for joint in Joints:
            res, joint_handle = vrep.simxGetObjectHandle(clientID, joint, vrep.simx_opmode_blocking)
            if res != vrep.simx_return_ok:
                print ('[ERROR] Handle Error for Joint %s' % (joint))
                return None
            else:
                Joint_Handles.append(joint_handle)
                print ('[INFO] Handle %d is set to Joint %s' % (joint_handle, joint))
        return Joint_Handles

    def set_joint_position_streaming(self, clientID, Joint_Handles):
        for joint_handle in Joint_Handles:
            vrep.simxGetJointPosition(clientID, joint_handle, vrep.simx_opmode_streaming)

    def set_joint_force_streaming(self, clientID, Joint_Handles):
        for joint_handle in Joint_Handles:
            vrep.simxGetJointForce(clientID, joint_handle, vrep.simx_opmode_streaming)

    def get_joint_position_streaming(self, clientID, Joint_Handles):
        Joint_Positions = list()
        for joint_handle in Joint_Handles:
            res, joint_position = vrep.simxGetJointPosition(clientID, joint_handle, vrep.simx_opmode_buffer)
            if res != vrep.simx_return_ok and res != vrep.simx_return_novalue_flag:
                print ('[ERROR] Get Position for Handle %d failed with Error Code %d' % (joint_handle, res))
                return None
            else:
                Joint_Positions.append(joint_position)
        return Joint_Positions

    def get_joint_force_streaming(self, clientID, Joint_Handles):
        Joint_Forces = list()
        for joint_handle in Joint_Handles:
            res, joint_force = vrep.simxGetJointForce(clientID, joint_handle, vrep.simx_opmode_buffer)
            if res != vrep.simx_return_ok and res != vrep.simx_return_novalue_flag:
                print ('[ERROR] Get Joint Force for Handle %d failed with Error Code %d' % (joint_handle, res))
                return None
            else:
                Joint_Forces.append(joint_force)
        return Joint_Forces

    def set_joint_position(self, clientID, Joint_Handles, TargetJoint_Position, CurrentJoint_Position):
        for idx, handle in enumerate(Joint_Handles):
            #if(abs(TargetJoint_Position[idx] - CurrentJoint_Position[idx]) > 0.005):
            vrep.simxSetJointTargetPosition(clientID, handle, TargetJoint_Position[idx], vrep.simx_opmode_oneshot)
        return

    def set_goal(self, x, y):
        self.goal_position = np.array([x,y])
    

    ####################### PAY ATTENTATION !!!!!!!!!! avaliable member attributes thant can be used to compute features and rewards #################
    #    prev_robot_position, curr_robot_position, curr_robot_orientation, curr_joint_position, action, distance, prev_distance  #

    def get_reward(self,coef_dist=50.0, coef_step=2.0, coef_rise=0.2, coef_h=0.02, coef_time=0.5):
        reward = 0
        # distance reduction
        reduced_distance_towards_goal = -self.distance_reward
        reward += coef_dist * reduced_distance_towards_goal
        print("dist: ", reward)

        # energy consumption
        il,ir = self.get_action(self.action)
        action_l = self.action4_single_leg[il]
        action_r = self.action4_single_leg[ir]
        reward_ = -(coef_step*(abs(action_l[0]) + abs(action_r[0])) + coef_rise*(action_l[1] + action_r[1]) + coef_h*(action_l[2] + action_r[2]))
        reward += reward_
        print("energy: ",reward_)
        
        # time
        reward += -coef_time
        #print("time: ",-coef_time)
        print("reward", reward)
        return reward  

    def get_states(self):
        pos = self.mot.get_position()
        states = 0
        acc = 3*3*3*11
        for leg_idx,tip_pos in enumerate(pos):
            #print(type(tf_2d))

            #print(tip_pos)
            x, y, z = self.tf_3d.local_to_global(tip_pos[2]/1000.0, tip_pos[1]/1000.0, tip_pos[0]/1000.0)
            #print("leg_idx:", leg_idx)
            #print("tip_pos: ",x, y, z)
            dist = (x+y+1)/np.sqrt(2)
            #dist = 100.0
            #if leg_idx == 0:
            #print("leg ",leg_idx," dist:", dist)
            height = tip_pos[2]

            if 0.0 < dist < 0.05:
                states += 0
                #print("leg ",leg_idx," 0")
            elif 0.05 < dist < 0.10:
                states += acc
                #print("leg ",leg_idx," 1")
            else:
                states += acc*2
                #print("leg ",leg_idx," 2")
            acc /= 3
        states += np.clip(int(self.obstacle.height*100),0,10)
        return int(states)

    def take_action(self):
        il,ir = self.get_action(self.action)
        print("action: ",self.action)
        #if il >=230
        #print("l: ",il,"r: ",ir)
        action_l = self.action4_single_leg[il]
        action_r = self.action4_single_leg[ir]
        print("action_l:", action_l)
        print("action_r:", action_r)

        self.mot.motion_set_x_speed(self.speed * np.cos(self.local_goal_direction))
        self.mot.motion_set_y_speed(self.speed * np.sin(self.local_goal_direction))
        self.mot.step_left.change_point(2, action_l[0])
        self.mot.rise_left.change_point(2, action_l[1])
        self.mot.step_right.change_point(2, action_r[0])
        self.mot.rise_right.change_point(2, action_r[1])
        self.mot.motion_set_front_h(action_l[2],action_r[2])

        joint_position = np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])
        pre_dist = self.prev_distance
        for t in np.arange(0.02, 1.0, 0.02):
            starttime=time.time()
            l1, l2, l3 = self.mot.motion_tick(t)
            joint_position[[0, 3, 6, 9]] = l1
            joint_position[[1, 4, 7, 10]] = l2
            joint_position[[2, 5, 8, 11]] = l3
            
            self.set_joint_position(self.clientID, self.Joint_Handles, joint_position, self.curr_joint_position)
            time.sleep(0.02)
            #print("dt:", time.time() - starttime)
        self.update()
        self.distance_reward = self.distance - pre_dist
        return 
        
    def get_action(self, action_index):
        action4_single_leg_num = len(self.action4_single_leg)
        print("action_index: ",action_index)
        left_leg_index = action_index//action4_single_leg_num
        right_leg_index = action_index%action4_single_leg_num
        return left_leg_index, right_leg_index

    

