#!/usr/bin/env python
import rospy
from sensor_msgs.msg import CameraInfo, Image
import numpy as np

import numpy as np
import cv2
from ros_numpy import numpify

from robot_helpers.srv import CreateFrameAtPose
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler, quaternion_from_matrix, rotation_matrix
from math import sin, cos


class CameraInfoReader:
  def __init__(self):
    rospy.init_node('get_transformations')
    self.data_to_use = input('choose number from 1.rob, 2.grip, 3.big_grip, 4.small_single_grip\n')
    print('you chose {}'.format(self.data_to_use))
    # self.data_to_use = 'rob'
    # self.data_to_use = 'grip'
    # self.data_to_use = 'big_grip'
    # self.data_to_use = 'small_single_grip'
    self.marker_data = {'1':{'name':'rob', 'sz':0.1, 'dict':cv2.aruco.DICT_5X5_100},
                   '2':{'name':'grip', 'sz':0.028, 'dict':cv2.aruco.DICT_4X4_100},
                   '3':{'name':'big_grip', 'sz':0.1, 'dict':cv2.aruco.DICT_5X5_100},
                   '4':{'name':'small_single_grip', 'sz':0.028, 'dict':cv2.aruco.DICT_4X4_100}}
    rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
    
  
  def get_parameters(self):
    msg = rospy.wait_for_message('/camera/color/camera_info', CameraInfo)
    self.K = np.array(msg.K).reshape(3, 3)
    self.D = np.array(msg.D).reshape(5, 1)
    np.save('/home/bass/ur5_rob_ws/src/perception/config/calibration_matrix.npy', self.K)
    np.save('/home/bass/ur5_rob_ws/src/perception/config/distortion_coefficients.npy', self.D)
    # rospy.signal_shutdown('Done')
  
  def image_callback(self, msg):
    image = numpify(msg)
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    self.get_parameters()
    ref_frame = self.pose_esitmation(image, self.marker_data[self.data_to_use]['dict'], self.K, self.D)
    cv2.imshow('ref_frame', ref_frame)
    cv2.waitKey(10)
  
  def rodrigues_to_quaternion(self, rvec):
      angle = np.linalg.norm(rvec)
      ax, ay, az = rvec / angle
      qx = ax * sin(angle/2)
      qy = ay * sin(angle/2)
      qz = az * sin(angle/2)
      qw = cos(angle/2)
      return np.array([qx, qy, qz, qw])
    
  
  def pose_esitmation(self, frame, aruco_dict_type, matrix_coefficients, distortion_coefficients):

    '''
    frame - Frame from the video stream
    matrix_coefficients - Intrinsic matrix of the calibrated camera
    distortion_coefficients - Distortion coefficients associated with your camera

    return:-
    frame - The frame with the axis drawn on it
    '''

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_type)
    parameters = cv2.aruco.DetectorParameters_create()


    corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, cv2.aruco_dict,parameters=parameters)
    
    # If markers are detected
    if len(corners) > 0:
        for i in range(0, len(ids)):
            # Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
            rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], self.marker_data[self.data_to_use]['sz'], matrix_coefficients,
                                                                       distortion_coefficients)
            # Draw a square around the markers
            cv2.aruco.drawDetectedMarkers(frame, corners) 

            # Draw Axis
            cv2.drawFrameAxes(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.1)
            tvec = np.array(tvec).flatten()
            rvec = np.array(rvec).flatten()
            print('rvec {}: {}'.format(ids[i],rvec))
            print('tvec {}: {}'.format(ids[i], tvec))
            if ids[i] == 3:
              id_3 = tvec
              self.create_frames(tvec, rvec, 'marker_3', 'camera_color_optical_frame')
              self.create_frames([0, 0.036, 0], [0, 0, 0], 'detected_tool', 'marker_3')
            if ids[i] == 4:
              id_4 = np.array(tvec)
            if ids[i] == 7:
              id_7 = np.array(tvec)
            if ids[i] == 2:
              id_2 = np.array(tvec)
            if self.marker_data[self.data_to_use]['name'] == 'rob':
              if tvec[2] > 1:
                self.create_frames(tvec, rvec, 'detected_base', 'camera_color_optical_frame')
            if self.marker_data[self.data_to_use]['name'] == 'big_grip':
              if tvec[2] <= 1.2:
                # print('Here')
                self.create_frames(tvec, rvec, 'marker_3', 'camera_color_optical_frame')
            if self.marker_data[self.data_to_use]['name']== 'small_single_grip':
              if tvec[2] <= 1:
                self.create_frames(tvec, rvec, 'marker_2', 'camera_color_optical_frame')
        
        if self.marker_data[self.data_to_use]['name'] == 'grip':
          print("horizontal  = ", np.linalg.norm(id_4 - id_3))
          print("vertical  = ", np.linalg.norm(id_4 - id_7))

    return frame

  
  def create_frames(self, tvec, rvec, new_frame, ref_frame):
    frame_pose = Pose()
    frame_pose.position.x = tvec[0]
    frame_pose.position.y = tvec[1]
    frame_pose.position.z = tvec[2]
    quat = self.rodrigues_to_quaternion(rvec)
    frame_pose.orientation.x = quat[0]
    frame_pose.orientation.y = quat[1]
    frame_pose.orientation.z = quat[2]
    frame_pose.orientation.w = quat[3]
    
    transformation_data = {
        "new_frame": String(new_frame),
        "ref_frame": String(ref_frame),
        "frame_pose": frame_pose
    }
    response = self.service_req(
        "/create_frame_at_pose", CreateFrameAtPose, inputs=transformation_data
    )
    data = response.result
  
  def service_req(self, name, service_type, **inputs):
      _ = rospy.wait_for_service(name)
      try:
          callable_service_func = rospy.ServiceProxy(name, service_type)
          response = callable_service_func(**inputs["inputs"])
          return response
      except rospy.ServiceException as e:
          print("Service Failed : {}".format(e))
    
    
    
  
  
    


if __name__ == '__main__':
  CameraInfoReader()
  rospy.spin()