import cv2
import numpy as np
import time
import cv2.aruco as aruco

from enum import IntEnum
class marker_id(IntEnum):
	ROBOT1 = 1
	ROBOT2 = 2
	ROBOT3 = 3
	GOAL = 4
	ORIGIN = 5
	OBSTACLE = 6

class Aruco_detector:
	'''
	def __init__(self, CameraMatrix, DistCoeffs, Video):

		# set camera parameters
		self.cameraMatrix = CameraMatrix
		self.distCoeffs = DistCoeffs

		# set video interface
		video = Video
		self.cap = cv2.VideoCapture(video)

		# set font for displaying text (below)  
		self.font = cv2.FONT_HERSHEY_SIMPLEX 
	'''

	def __init__(self):

		# set camera parameters
		self.cameraMatrix =  np.array([
			[680.9407, 0, 317.4899],
			[0, 682.4478, 240.1040],
			[0, 0, 1]
			])

		#self.cameraMatrix = np.transpose(self.cameraMatrix)
		self.distCoeffs = np.array([0.2834, -0.7486, 0, 0])
		# set video interface
		video = "/dev/video0"
		self.cap = cv2.VideoCapture(video)

		# set font for displaying text (below)  
		self.font = cv2.FONT_HERSHEY_SIMPLEX


	def get_aruco_posture(self, id):
		ret, frame = self.cap.read()
		aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_50)
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
				if i == marker_id.ROBOT1:
					cv2.putText(frame, "robot1(Id: " + str(i) + ")", tuple(position4Text), self.font, 0.7, (0,255,0),2,cv2.LINE_AA)
					#print("Robot marker detected.")
				elif i == marker_id.ROBOT2:
					cv2.putText(frame, "robot2(Id: " + str(i) + ")", tuple(position4Text), self.font, 0.7, (0,255,0),2,cv2.LINE_AA)
				elif i == marker_id.ROBOT3:
					cv2.putText(frame, "robot3(Id: " + str(i) + ")", tuple(position4Text), self.font, 0.7, (0,255,0),2,cv2.LINE_AA)
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
					print("Robot1 marker not detected!")
				if id == 2:
					print("Robot2 marker not detected!")
				if id == 3:
					print("Robot3 marker not detected!")
				elif id == 4:
					print("Goal marker not detected! ")
				elif id == 5:
					print("Origin marker not detected!")
				elif id == 6:
					print("Obstacle marker not detected!")

		
		else:
			print("No marker detected!")
			
		# display the frame
		cv2.imshow("frame", frame)
		return id_is_detected, transform_matrix

		

	def some_method(self):
		ret, frame = self.cap.read()
		print(frame.shape)
		cv2.imshow("frame", frame)
			
	def deconstructor(self):
		self.cap.release()
		cv2.destroyAllWindows()



def main():
	# Build aruco detector
	aruco_detector = Aruco_detector()
	
	# first find the fixed goal posture, origin, obstacle posture for the playground
	'''
	result4goal, transform4goal = aruco_detector.get_aruco_posture(marker_id.GOAL)
	result4origin, transform4origin = aruco_detector.get_aruco_posture(marker_id.ORIGIN)
	result4obstacle, transform4obstacle = aruco_detector.get_aruco_posture(marker_id.OBSTACLE)

	
	if not (result4goal and result4origin and result4obstacle):
		raise Exception("Position of goal or origin or obstacle can not be detected") 
	'''
	
	while True:
		aruco_detector.get_aruco_posture(marker_id.ORIGIN)
		cv2.waitKey(1)
	
	
	'''
	while True:
		aruco_detector.some_method()
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break
	'''
	aruco_detector.deconstructor()



if __name__ =="__main__":
	main()