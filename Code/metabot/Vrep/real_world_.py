import numpy as np
import math
import time
import os
from Motion.motion import motion
from TF.tf_2d import tf_2d
from TF.tf_3d import tf_3d
from Aruco.aruco_detector import Aruco_detector, marker_id
from API.metabotAPI import metabotAPI
###########################################PREDEFINED VARIABLES#########################################################################################
              
class obstacle:
    def __init__(self, transform):
        self.transform = transform
        self.height = 0
        self.length = 0
        self.width = 0


class environment:
    def __init__(self):
        self.tf_2d = tf_2d()
        #self.tf_3d = tf_3d()
        self.tf_r2w = tf_3d()
        self.tf_w2o = tf_3d()
        
        self.initial_robot_position = None
        self.curr_robot_transform = None
        self.prev_robot_position = None
        self.action = 0
        
        self.initial_joint_position = np.array([0.0, 0.0, 0.0, 0.0, -29.76, -29.76, -29.76, -29.76, -123.87, -123.87, -123.87, -123.87]) 


        self.incline_threshold = 2.8
        self.is_incline = False
        self.states = None
        self.distance = 0.0
        self.prev_distance = 0.0
        self.distance_reward = 0.0
        self.local_goal_direction = 0.0
        # public variables to store goal position and obstable positions and heights
        self.support = 0.5
        self.speed = 200.0
        self.mot = motion()
        self.setup_motion()

        self.action4_single_leg = []
        step = [-0.5,0.0,0.5]
        rise = [1.0,4.0,7.0]
        front_h = [0.0,45.0,90.0]
        for i in range(len(step)):
            for j in range(len(rise)):
                for k in range(len(front_h)):
                    self.action4_single_leg.append([step[i],rise[j],front_h[k]])

        #initialize Aruco detector
        self.aruco_detector = Aruco_detector()
        
        # set a proper origin
        print("Waiting for setting origin...")
        while True:
            result, transform = self.aruco_detector.get_aruco_posture(marker_id.ORIGIN)
        
            if result:
                isOK = input("Dose this origin look OK? If OK press y to finish setting origin:")
                if isOK == "y":
                    print("Setting origin finished")
                    break

        print("Origin's position with respect to camera is: ")
        print(transform[0:3,3])
        self.transform4origin_inv = np.linalg.inv(transform)

        '''
        # set a proper goal
        print("Waiting for setting goal...")
        while True:
            result, transform = self.aruco_detector.get_aruco_posture(marker_id.GOAL)
        
            if result:
                isOK = input("Dose this goal look OK? If OK press y to finish setting goal:")
                if isOK == "y":
                    print("Setting goal finished")
                    break
        transform_to_origin = np.dot(self.transform4origin_inv, transform)
        self.goal_position = transform_to_origin[0:2,3]
        print("Goal's 2d position with respect to origin is: ")
        print(self.goal_position)
        '''

        # set a proper obstacle
        '''
        print("Waiting for setting obstacle...")
        while True:
            result, transform = self.aruco_detector.get_aruco_posture(marker_id.OBSTACLE)
        
            if result:
                isOK = input("Dose this obstacle look OK? If OK press y to finish setting obstacle:")
                if isOK == "y":
                    print("Setting obstacle finished")
                    break
        transform_to_origin = np.dot(self.transform4origin_inv, transform)
        self.obstacle = obstacle(transform_to_origin)
        self.tf_w2o.set_tf_matrix(self.obstacle.transform)
        print("Obstacle's transform with respect to origin is: ")
        print(self.obstacle.transform)
        '''
        print("environment.aruco_detector initialized")

        self.metabotapi = metabotAPI()
        print("environment.metabotapi initialized")


    def get_robot_pose(self):
        
        # get robot marker1 position
        result, transform = self.aruco_detector.get_aruco_posture(marker_id.ROBOT1)
        if result:
            transform_to_origin = np.dot(self.transform4origin_inv, transform)
            marker_position = transform_to_origin[0:3,3]
            #print("Current detected robot position is: ")
            #print(robot_position)
            robot_marker1_position = marker_position

        # get robot marker2 position
        result, transform = self.aruco_detector.get_aruco_posture(marker_id.ROBOT2)
        if result:
            transform_to_origin = np.dot(self.transform4origin_inv, transform)
            marker_position = transform_to_origin[0:3,3]
            #print("Current detected robot position is: ")
            #print(robot_position)
            robot_marker2_position = marker_position

        # get robot marker3 position
        result, transform = self.aruco_detector.get_aruco_posture(marker_id.ROBOT3)
        if result:
            transform_to_origin = np.dot(self.transform4origin_inv, transform)
            marker_position = transform_to_origin[0:3,3]
            #print("Current detected robot position is: ")
            #print(robot_position)
            robot_marker3_position = marker_position

        return robot_marker1_position, robot_marker2_position, robot_marker3_position
            
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
        # robot joint go to initial values

        self.metabotapi.motor_go_myway(self.initial_joint_position)
        self.action = 0
        self.update()
        self.prev_distance = self.distance
        self.is_incline = False

    def update(self):
        x_axis, origin, y_axis = self.get_robot_pose()
        self.cal_trans(origin, x_axis, y_axis)
        if self.curr_robot_transform is None:
            return

        self.prev_distance = self.distance
        self.tf_r2w.set_tf_matrix(self.curr_robot_transform)
        
        robot_orientation = self.tf_r2w.trans_matrix[0:3,0:3]*np.matrix([0.0,0.0,1.0]).T

        robot_incline_angle = np.arccos(np.dot(np.array([robot_orientation[0,0], robot_orientation[1,0], robot_orientation[2,0]]), np.array([0.0, 0.0, 1.0])) / (np.linalg.norm(np.array([robot_orientation[0,0], robot_orientation[1,0], robot_orientation[2,0]])) * np.linalg.norm(np.array([0.0, 0.0, 1.0]))))
        if robot_incline_angle > self.incline_threshold:
            self.is_incline = True
        robot_direction_3d = self.tf_r2w.trans_matrix[0:3,0:3]*np.matrix([1.0,0.0,0.0]).T
        robot_direction = np.arctan2(robot_direction_3d[1,0], robot_direction_3d[0,0])
        self.tf_2d.set_tf(self.curr_robot_transform[0,3], self.curr_robot_transform[1,3], robot_direction)
        local_goal_x, local_goal_y = self.tf_2d.global_to_local(self.goal_position[0], self.goal_position[1])
        local_goal_y = -local_goal_y
        self.local_goal_direction = np.arctan2(local_goal_y, local_goal_x)
        print("curr_robot_position:", self.curr_robot_position)
        print("robot_direction:", robot_direction)
        print("local_goal_direction:", local_goal_direction)
        print("x:", local_goal_x)
        print("y:", local_goal_y)
        self.distance = np.linalg.norm([local_goal_x, local_goal_y])
        #self.states = self.get_states()


    def set_goal(self, x, y):
        self.goal_position = np.array([x,y])

    def get_reward(self,coef_dist=50,coef_vel=10,coef_dev_hor=1.5,coef_dev_ver=5,coef_energy=2.5,coef_time=0.5):
        reward = 0
        # distance reduction
        reduced_distance_towards_goal = -self.distance_reward
        reward += coef_dist * reduced_distance_towards_goal
        # energy consumption
        il,ir = self.get_action(self.action,48)
        action_l = self.action4_single_leg[il]
        action_r = self.action4_single_leg[ir]
        reward_ = -(coef_step*(abs(action_l[0]) + abs(action_r[0])) + coef_rise*(action_l[1] + action_r[1]) + coef_h*(action_l[2] + action_r[2]))
        reward += reward_
        print("energy: ",reward_)

        # time
        reward += -coef_time
        print("time: ",-coef_time)
        return reward  

    def get_states(self):
        pos = self.mot.get_position()
        states = 0
        acc = 3*3*3*11
        for leg_idx,tip_pos in enumerate(pos):

            #print(tip_pos)
            x_world, y_world, z_world = self.tf_r2w.local_to_global(tip_pos[2]/1000.0, tip_pos[1]/1000.0, tip_pos[0]/1000.0)
            x_obs, y_obs, z_obs = self.tf_w2o.local_to_global(x_world, y_world, z_world)

            if x_obs < 0.2 and x_obs >0 and y_obs <0:
                dist = -y_obs

            height = z_world

            if 0.0 < dist < 0.02:
                states += 0
                #print("leg ",leg_idx," 0")
            elif 0.02 < dist < 0.05:
                states += acc
                #print("leg ",leg_idx," 1")
            else:
                states += acc*2
                #print("leg ",leg_idx," 2")
            acc /= 3
        states += np.clip(int(self.obstacle.height*100),0,10)
        return int(states)

    def take_action(self):
        def get_action(action_index,action4_single_leg_num):
            left_leg_index = action_index//action4_single_leg_num
            right_leg_index = action_index%action4_single_leg_num
            return left_leg_index, right_leg_index

        il,ir = get_action(self.action,48)
        #if il >=230
        #print("l: ",il,"r: ",ir)
        action_l = self.action4_single_leg[il]
        action_r = self.action4_single_leg[ir]
        print("action_l:", action_l)
        print("action_r:", action_r)
        
        self.mot.motion_set_x_speed(self.speed * np.cos(self.local_goal_direction))
        self.mot.motion_set_y_speed(self.speed * np.sin(self.local_goal_direction))
        #self.mot.motion_set_x_speed(166.67)
        #self.mot.motion_set_y_speed(0.0)
        self.mot.step_left.change_point(2, action_l[0])
        self.mot.rise_left.change_point(2, action_l[1])
        self.mot.step_right.change_point(2, action_r[0])
        self.mot.rise_right.change_point(2, action_r[1])
        self.mot.motion_set_front_h(action_l[2],action_r[2])

        joint_position = np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])
        pre_dist = self.prev_distance
        for t in np.arange(0.02, 1.0, 0.02):
            l1, l2, l3 = self.mot.motion_tick(t)
            joint_position[[0, 1, 2, 3]] = l1
            joint_position[[4, 5, 6, 7]] = l2
            joint_position[[8, 9, 10, 11]] = l3
            self.update()
            print("desired joint position:", joint_position*180/np.pi)
            print("=====================================================")
            self.metabotapi.motor_go_myway(joint_position*180/np.pi)

        self.distance_reward = self.distance - pre_dist
        return
        
    def cal_trans(self, origin, x_axis, y_axis):
        if origin is None or x_axis is None or y_axis is None:
            return
        pos = origin
        x_direction = x_axis - origin
        x_direction = x_direction / np.linalg.norm(x_direction)
        y_direction = y_axis - origin
        y_direction = y_direction / np.linalg.norm(y_direction)
        z_direction = np.cross(x_direction, y_direction)
        self.curr_robot_transform = np.matrix([[x_direction[0], y_direction[0], z_direction[0], pos[0]], 
                                               [x_direction[1], y_direction[1], z_direction[1], pos[1]],
                                               [x_direction[2], y_direction[2], z_direction[2], pos[2]],
                                               [0, 0, 0, 1]])
        

    

