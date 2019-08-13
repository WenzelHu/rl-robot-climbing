import cv2
import numpy as np
import time
import cv2.aruco as aruco
import threading
from collections import deque

from enum import IntEnum
class marker_id(IntEnum):
    ROBOT = 0
    GOAL = 2
    ORIGIN = 4
    OBSTACLE = 5

class Aruco_detector(threading.Thread):
    def __init__(self):
        # initialize thread
        threading.Thread.__init__(self)
        #self.curr_corners = None
        #self.prev_corners = None
        self.curr_time = 0
        #self.curr_robot_transform = None
        self.robot_transforms_queue = deque([],5)
        # set camera parameters
        self.cameraMatrix =  np.array([
            [1.0380e3, 0, 636.5514],
            [0, 1.0350e3, 369.5972],
            [0, 0, 1]
            ])
        
        #self.cameraMatrix = np.transpose(self.cameraMatrix)
        self.distCoeffs = np.array([0.1747, -0.4743, 0, 0])
        # set video interface
        video = "/dev/video0"
        self.cap = cv2.VideoCapture(video)

        # set font for displaying text (below)  
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.frame = None
    
    def run(self):
        print("Start threading for robot aruco detection.")
        self.update_robot_pose()
    
    
    def get_origin_pose(self):
        ret, self.frame = self.cap.read()
        aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
        arucoParams = aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = aruco.detectMarkers(self.frame, aruco_dict, parameters = arucoParams)
        transform_matrix = None
        if ids is not None and [marker_id.ORIGIN] in list(ids):
            index_in_list = list(ids).index([marker_id.ORIGIN])
            corners_array = corners[index_in_list][0]
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers([corners_array], 0.05, self.cameraMatrix, self.distCoeffs) 
            print(corners[index_in_list])
            aruco.drawAxis(self.frame, self.cameraMatrix, self.distCoeffs, rvec[0, :, :], tvec[0, :, :], 0.03)
            aruco.drawDetectedMarkers(self.frame, [corners[index_in_list]])
            
            _, max_y_index = np.argmax(corners_array, axis=0)
            lowest_corner = corners_array[max_y_index, :]
            position4Text = np.copy(lowest_corner)
            position4Text[1] +=20
            cv2.putText(self.frame, "origin(Id: " + str(int(marker_id.ORIGIN)) + ")", tuple(position4Text), self.font, 0.7, (0,255,0),2,cv2.LINE_AA)
            transform_matrix = np.zeros([4,4])
            transform_matrix[0:3, 3] = tvec[0,0,:]
            rotationMatrix, _ = cv2.Rodrigues(rvec[0,0,:])
            transform_matrix[0:3,0:3] = rotationMatrix
            transform_matrix[3, 3] = 1
        else:
            print("no origin aruco detected") 
        return transform_matrix

    '''
    def low_pass(self, corners, dt):
        filtered_corners = np.copy(corners)
        if self.prev_corners is not None:
            cut_off = 0.8
            RC = 1.0/(cut_off*2*3.14)
            alpha = dt/(RC + dt)
            for i, element in enumerate(filtered_corners):
                for j, _ in enumerate(element):
                    filtered_corners[i,j] = self.prev_corners[i,j] + alpha*(corners[i,j] - self.prev_corners[i,j])

        return filtered_corners
    '''
    def update_robot_pose(self):
        while True:
            ret, self.frame = self.cap.read()
            aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
            arucoParams = aruco.DetectorParameters_create()
            corners, ids, rejectedImgPoints = aruco.detectMarkers(self.frame, aruco_dict, parameters = arucoParams)
            robot_is_detected = False
            transform_matrix = None
            if ids is not None:
                for i in ids:
                    index_in_list = list(ids).index(i)
                    corners_array = corners[index_in_list][0]
                    '''
                    #apply low pass to robot corners
                    if i == marker_id.ROBOT:
                        dt = time.time() - self.curr_time 
                        self.curr_time = time.time()
                        corners_array = self.low_pass(corners_array, dt)
                        self.prev_corners = corners_array
                    '''    

                    rvec, tvec, _ = aruco.estimatePoseSingleMarkers([corners_array], 0.05, self.cameraMatrix, self.distCoeffs) 
                    
                    aruco.drawAxis(self.frame, self.cameraMatrix, self.distCoeffs, rvec[0, :, :], tvec[0, :, :], 0.03)
                    aruco.drawDetectedMarkers(self.frame, [corners[index_in_list]])
                    _, max_y_index = np.argmax(corners_array, axis=0)
                    lowest_corner = corners_array[max_y_index, :]
                    position4Text = np.copy(lowest_corner)
                    position4Text[1] +=20
                    
                    # draw conners and axis onto frame
                    if i == marker_id.ROBOT:
                        cv2.putText(self.frame, "robot(Id: " + str(i) + ")", tuple(position4Text), self.font, 0.7, (0,255,0),2,cv2.LINE_AA)    
                    elif i == marker_id.GOAL:
                        cv2.putText(self.frame, "goal(Id: " + str(i) + ")", tuple(position4Text), self.font, 0.7, (0,255,0),2,cv2.LINE_AA)
                        #print("Goal marker detected.")
                    elif i == marker_id.ORIGIN:
                        cv2.putText(self.frame, "origin(Id: " + str(i) + ")", tuple(position4Text), self.font, 0.7, (0,255,0),2,cv2.LINE_AA)
                        #print("Origin marker detected.")
                    elif i == marker_id.OBSTACLE:
                        cv2.putText(self.frame, "obstacle(Id: " + str(i) + ")", tuple(position4Text), self.font, 0.7, (0,255,0),2,cv2.LINE_AA)
                        #print("Obstacle marker detected.")

                    # output the transform for robot
                    if i == marker_id.ROBOT:
                        robot_is_detected = True
                        transform_matrix = np.zeros([4,4])
                        transform_matrix[0:3, 3] = tvec[0,0,:]
                        rotationMatrix, _ = cv2.Rodrigues(rvec[0,0,:])
                        transform_matrix[0:3,0:3] = rotationMatrix
                        transform_matrix[3, 3] = 1
                        self.robot_transforms_queue.append(transform_matrix) 

                        #print(" And we need to output its transformation matrix with respect to camera:")
                        #print(" ", transform_matrix)
                    

                # if the transform that we want is not detected, output some debug information and tranform matrix should be none and result is false
                if not robot_is_detected:
                    print("Robot marker not detected!")            
            else:
                print("No marker detected!")
    
    
    '''
    def get_aruco_posture(self, id):
        ret, frame = self.cap.read()
        aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
        arucoParams = aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = aruco.detectMarkers(frame, aruco_dict, parameters = arucoParams)
        id_is_detected = False
        transform_matrix = None

        if ids is not None:
            for i in ids:
                index_in_list = list(ids).index(i)

                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[index_in_list], 0.05, self.cameraMatrix, self.distCoeffs) 
                
                #print(tvec)
                aruco.drawAxis(frame, self.cameraMatrix, self.distCoeffs, rvec[0, :, :], tvec[0, :, :], 0.03)
                aruco.drawDetectedMarkers(frame, [corners[index_in_list]])
                corners_array = corners[index_in_list][0]
                _, max_y_index = np.argmax(corners_array, axis=0)
                lowest_corner = corners_array[max_y_index, :]
                position4Text = np.copy(lowest_corner)
                position4Text[1] +=20
                
                # draw conners and axis onto frame
                if i == marker_id.ROBOT:
                    cv2.putText(frame, "robot(Id: " + str(i) + ")", tuple(position4Text), self.font, 0.7, (0,255,0),2,cv2.LINE_AA)
                    #print("Robot marker detected.")
                elif i == marker_id.GOAL:
                    cv2.putText(frame, "goal(Id: " + str(i) + ")", tuple(position4Text), self.font, 0.7, (0,255,0),2,cv2.LINE_AA)
                    #print("Goal marker detected.")
                elif i == marker_id.ORIGIN:
                    cv2.putText(frame, "origin(Id: " + str(i) + ")", tuple(position4Text), self.font, 0.7, (0,255,0),2,cv2.LINE_AA)
                    #print("Origin marker detected.")
                elif i == marker_id.OBSTACLE:
                    cv2.putText(frame, "obstacle(Id: " + str(i) + ")", tuple(position4Text), self.font, 0.7, (0,255,0),2,cv2.LINE_AA)
                    #print("Obstacle marker detected.")

                # output the transform that we want
                if i == id:
                    id_is_detected = True
                    transform_matrix = np.zeros([4,4])
                    transform_matrix[0:3, 3] = tvec[0,0,:]
                    rotationMatrix, _ = cv2.Rodrigues(rvec[0,0,:])
                    transform_matrix[0:3,0:3] = rotationMatrix
                    transform_matrix[3, 3] = 1
                    #print(" And we need to output its transformation matrix with respect to camera:")
                    #print(" ", transform_matrix)
                

            # if the transform that we want is not detected, output some debug information and tranform matrix should be none and result is false
            if not id_is_detected:
                if id == 1:
                    print("Robot marker not detected!")
                elif id == 2:
                    print("Goal marker not detected! ")
                elif id == 3:
                    print("Origin marker not detected!")
                elif id == 4:
                    print("Obstacle marker not detected!")

        
        else:
            print("No marker detected!")
            
        # display the frame
        cv2.imshow("frame", frame)
        cv2.waitKey(1)
        return id_is_detected, transform_matrix
        
        

    def some_method(self):
        ret, frame = self.cap.read()
        print(frame.shape)
        cv2.imshow("frame", frame)
    '''        
    def deconstructor(self):
        self.cap.release()
        cv2.destroyAllWindows()
    
def main():
    # Build aruco detector
    aruco_detector = Aruco_detector()
    
    # setting origin
    transform4origin = None
    
    
    print("Waiting for setting origin...")
    while True:
        transform4origin = aruco_detector.get_origin_pose()
        cv2.imshow("frame", aruco_detector.frame)
        cv2.waitKey(1)
        if transform4origin is not None:
            isOK = input("Dose this origin look OK? If OK press y to finish setting origin:")
            if isOK == "y":
                print("Setting origin finished")
                break
    transform4origin_inv = np.linalg.inv(transform4origin)
    
    # start the updating robot pose threading
    aruco_detector.start()

    while True:
        transforms_ = np.copy(aruco_detector.robot_transforms_queue)
        if transforms_.shape[0] < 5:
        	continue
        transforms = list()
        transforms.append(transforms_[0].flatten())
        transforms.append(transforms_[1].flatten())
        transforms.append(transforms_[2].flatten())
        transforms.append(transforms_[3].flatten())
        transforms.append(transforms_[4].flatten())
        cluster1 = transforms[0]
        cluster2 = transforms[4]
        transforms = np.array(transforms)
        pre_label = np.array([0, 0, 0, 0, 0])
        label = np.array([2, 2, 2, 2, 2])
        while True:
            for i in range(5):
                if np.linalg.norm(cluster1-transforms[i,:]) < np.linalg.norm(cluster2-transforms[i,:]):
                    label[i] = 1
                else:
                    label[i] = 2
            cluster1 = np.average(transforms[np.where(label == 1)[0].tolist(),:], axis = 0)
            cluster2 = np.average(transforms[np.where(label == 2)[0].tolist(),:], axis = 0)
            if (pre_label==label).all():
                break
            pre_label = label
        label1 = np.where(label == 1)[0].tolist()
        label2 = np.where(label == 2)[0].tolist()
        if len(label1) >= 3:
            transform = np.reshape(transforms[label1[-1],:], (4, 4))
        else:
            transform = np.reshape(transforms[label2[-1],:], (4, 4))
        transform4robot = np.dot(transform4origin_inv, transform)
        x = np.dot(np.linalg.inv(transform4robot[0:3,0:3]), np.array([1,0,0]))
        y = np.dot(np.linalg.inv(transform4robot[0:3,0:3]), np.array([0,1,0]))
        z = np.dot(np.linalg.inv(transform4robot[0:3,0:3]), np.array([0,0,1]))
        print("=======================================")
        print("x:",x)
        print("y:",y)
        print("z:",z)

        cv2.imshow("frame", aruco_detector.frame)
        cv2.waitKey(1)

    
    '''

    while True:
        _, transform4robot = aruco_detector.get_aruco_posture(marker_id.ROBOT)
        
        if transform4robot is not None:
            #transform4robot_inv = np.linalg.inv(transform4robot)
            transform4robot = np.dot(transform4origin_inv, transform4robot)
            x = np.dot(np.linalg.inv(transform4robot[0:3,0:3]), np.array([1,0,0]))
            y = np.dot(np.linalg.inv(transform4robot[0:3,0:3]), np.array([0,1,0]))
            z1 = np.dot(np.linalg.inv(transform4robot[0:3,0:3]), np.array([0,0,1]))
            z2 =  np.cross(x,y)
            print("=======================================")
            print(transform4robot)
            print("x:",x)
            print("y:",y)
            print("z1:",z1)
            print("z2:",z2)

        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    '''
    
    
    



if __name__ =="__main__":
    main()