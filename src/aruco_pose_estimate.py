import rospy
import numpy as np
import tf
import cv2
import cv2.aruco as aruco
import math


#The output of translation vector is in centimeters


rospy.init_node('talker', anonymous=True)

br=tf.TransformBroadcaster()

marker_size=10 #in centimeters

cap = cv2.VideoCapture(1)
rvec=np.zeros(3)
tvec=np.zeros(3)
_objpoints=np.zeros(4)



def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


def rotationMatrixToEulerAngles(R) :
 
    assert(isRotationMatrix(R))
     
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
     
    singular = sy < 1e-6
 
    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
 
    return np.array([x, y, z])

# cap.set(cv2.CAP_PROP_FRAME_WIDTH,640)
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
while(True):
    
	ret, frame = cap.read()

    
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
	parameters =  aruco.DetectorParameters_create()
	# print(parameters)
	corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
	copy=frame.copy()
	# frame_markers = aruco.drawDetectedMarkers(copy, corners, ids)
	
	camera_matrix   = np.loadtxt('/home/msamogh/Desktop/indoor_localisation_using_aruco/'+'cameraMatrix.txt', delimiter=',')
	camera_distortion   = np.loadtxt('/home/msamogh/Desktop/indoor_localisation_using_aruco/'+'cameraDistortion.txt', delimiter=',')

	
	
	if ids is not None :

		rvec,tvec,_objpoints = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)

		# print(rvec)
		# print(ids)

		print(len(rvec))

		aruco.drawDetectedMarkers(copy, corners)

		for i in range(len(ids)) :

			print(rvec[i])
		
			rot_from_camera_to_marker=np.matrix(cv2.Rodrigues(rvec[i])[0])
		
			aruco.drawAxis(copy, camera_matrix, camera_distortion, rvec[i], tvec[i], 10)
			print(np.matrix(cv2.Rodrigues(rvec[i])[0]))
		
			roll_marker, pitch_marker, yaw_marker = rotationMatrixToEulerAngles(rot_from_camera_to_marker)

			rot_from_camera_to_marker=tf.transformations.euler_matrix(roll_marker,pitch_marker,yaw_marker,'sxyz')

			transform_from_camera_to_marker=tf.transformations.quaternion_from_matrix(rot_from_camera_to_marker)

			
			tuple_now = (tvec[i][0][0],tvec[i][0][1],tvec[i][0][2])
			
			

			# print(rot_from_camera_to_marker.shape)
			
			transform_from_marker_to_camera=tf.transformations.quaternion_inverse(transform_from_camera_to_marker)

			br.sendTransform(tuple_now,transform_from_marker_to_camera,rospy.Time.now(),"marker"+str(i),"world")




			# rot_from_marker_to_camera=np.transpose(rot_from_camera_to_marker)

			# roll_camera, pitch_camera, yaw_camera = rotationMatrixToEulerAngles(rot_from_marker_to_camera)

			# print(roll_camera*180/3.14,pitch_camera*180/3.14,yaw_camera*180/3.14)

			# print(roll_marker*180/3.14,pitch_marker*180/3.14,yaw_marker*180/3.14)

			print("translation vector is" + str(tvec[i]))
	
	cv2.imshow('batao_batao',copy)
	

	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
