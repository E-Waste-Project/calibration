#!/usr/bin/env python
import rospy
from sensor_msgs.msg import CameraInfo, Image
import numpy as np

import numpy as np
import cv2
from ros_numpy import numpify

from robot_helpers.srv import CreateFrameAtPose, LookupTransform
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler, quaternion_matrix, quaternion_from_matrix, euler_matrix, rotation_matrix, euler_from_quaternion
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
    self.trackbar_limits = {'x': 500, 'rx': 500,
                            'y': 500, 'ry': 500,
                            'z': 500, 'rz': 500}
    self.create_camera = False
    
    self.detected_base = [[ 0.08195365, -0.00050111, -0.14764314], [-0.50428208, -0.49966274, -0.49110353, -0.50483071]]
    trans = np.array(self.detected_base[0])
    quat = np.array(self.detected_base[1])
    euler_ = euler_from_quaternion(quat)
    print(trans)
    print(euler_)
    # self.create_frames(trans, quat, 'aruco_base', 'base_link', quat=True)

    self.mode = None
    self.started = False
    self.collection_request_publisher = rospy.Publisher('/collect_data_request', String, queue_size=1)
    rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
    
  
  def get_parameters(self):
    msg = rospy.wait_for_message('/camera/color/camera_info', CameraInfo)
    self.K = np.array(msg.K).reshape(3, 3)
    self.D = np.array(msg.D).reshape(5, 1)
    np.save('/home/bass/ur5_rob_ws/src/perception/config/calibration_matrix.npy', self.K)
    np.save('/home/bass/ur5_rob_ws/src/perception/config/distortion_coefficients.npy', self.D)
    # rospy.signal_shutdown('Done')
  
  def image_callback(self, msg):
    if not self.started:
      win_name = 'adjust_frame'
      cv2.namedWindow(win_name)
      self.win_name = win_name
      for key, val in self.trackbar_limits.items():
          cv2.createTrackbar(key, self.win_name, val, 1000, lambda x: None)
      self.get_parameters()
      self.started = True
    image = numpify(msg)
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if self.mode == 's':
      # print("here")
      ref_frame = self.pose_esitmation(image, self.marker_data[self.data_to_use]['dict'], self.K, self.D)
      image = ref_frame
    elif self.mode == 'm':
      self.adjust_frame_manually()
    cv2.imshow(self.win_name, image)
    val = cv2.waitKey(10) & 0xFF
    if val == ord('a'):
      print("sending collection request")
      self.collection_request_publisher.publish(String('collect'))
    elif val == ord('c'):
      print("calculating hand eye transformation")
      self.calculate_hand_eye_transformation()
    elif val == ord('m'):
      self.mode = 'm'
    elif val == ord('s'):
      self.data_to_use = '4'
      self.mode = 's'
    elif val == ord('r'):
      gc_t, gc_r = self.get_transform('marker_3', 'camera_color_optical_frame') # gribber wrt camera
      self.create_frames(gc_t, gc_r, 'camera_color_optical_frame', 'grip_palm', quat=True)
    elif val == ord('f'):
      self.data_to_use = '1'
      self.mode = 's'
    elif val == ord('p'):
      gc_t, gc_r = self.get_transform('base_link', 'detected_base')
      print(gc_t, gc_r)
      euler_ = euler_from_quaternion(gc_r)
      print(gc_t)
      print(euler_)
    elif val == ord('b'):
      self.create_camera = True
      self.data_to_use = '1'
      self.mode = 's'
      
  def adjust_frame_manually(self):
    vals = {}
    original_vals = {}
    for key in self.trackbar_limits.keys():
      original_vals[key] = cv2.getTrackbarPos(key, self.win_name)
      vals[key] = (0.001 * (cv2.getTrackbarPos(key, self.win_name) - 500))
      if 'r' in key:
        vals[key] *= np.pi
    self.trackbar_limits = original_vals
    quat = quaternion_from_euler(vals['rx'], vals['ry'], vals['rz'])
    trans = [vals['x'], vals['y'], vals['z']]
    self.create_frames(trans, quat,'camera_color_optical_frame', 'marker_3',quat=True)
    
  
  def rodrigues_to_quaternion(self, rvec):
      angle = np.linalg.norm(rvec)
      ax, ay, az = rvec / angle
      qx = ax * sin(angle/2)
      qy = ay * sin(angle/2)
      qz = az * sin(angle/2)
      qw = cos(angle/2)
      return np.array([qx, qy, qz, qw])
  
  def quaternion_to_rodrigues(self, quat):
    qx, qy, qz, w = quat[0], quat[1], quat[2], quat[3]
    angle = np.arccos(w)*2
    ax = qx / sin(angle/2)
    ay = qy / sin(angle/2)
    az = qz / sin(angle/2)
    rx = ax * angle
    ry = ay * angle
    rz = az * angle
    return np.array([rx, ry, rz])
    
  
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
            # print('rvec {}: {}'.format(ids[i],rvec))
            # print('tvec {}: {}'.format(ids[i], tvec))
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
                if self.create_camera:
                  quat = self.rodrigues_to_quaternion(rvec)
                  rot_mat = quaternion_matrix(quat)
                  rot_mat[:3,-1] = np.copy(tvec)
                  new_rot_mat = np.linalg.inv(rot_mat)
                  new_tvec = new_rot_mat[:3, -1]
                  new_mat = np.eye(4)
                  new_mat[:3,:3] = np.copy(new_rot_mat[:3,:3])
                  new_quat = quaternion_from_matrix(new_mat)
                  # print(new_tvec)
                  # print(new_quat)
                  self.create_frames(new_tvec, new_quat, 'camera_color_optical_frame', 'aruco_base',quat=True)
                else:
                  self.create_frames(tvec, rvec, 'detected_base', 'camera_color_optical_frame')
            if self.marker_data[self.data_to_use]['name'] == 'big_grip':
              if tvec[2] <= 1.2:
                # print('Here')
                self.create_frames(tvec, rvec, 'marker_3', 'camera_color_optical_frame')
            if self.marker_data[self.data_to_use]['name']== 'small_single_grip':
              if tvec[2] <= 1.2:
                quat = self.rodrigues_to_quaternion(rvec)
                mat = quaternion_matrix(quat)
                rot_mat2 = rotation_matrix(np.pi/2, [0, 0, 1])
                rot_mat1 = rotation_matrix(np.pi/2, [1, 0, 0])
                rot_mat1[:3,-1] = np.array([0, 0, -0.05])
                mat[:3,-1] = tvec
                new_mat = np.dot(mat, rot_mat2)
                new_mat = np.dot(new_mat, rot_mat1)
                new_tvec = np.copy(new_mat[:3,-1])
                new_mat[:3, -1] = np.zeros(3,)
                new_quat = quaternion_from_matrix(new_mat)
                new_rvec = self.quaternion_to_rodrigues(new_quat)
                cv2.drawFrameAxes(frame, matrix_coefficients, distortion_coefficients, new_rvec, tvec, 0.1)
                # self.create_frames(tvec, quat, 'marker_3', 'camera_color_optical_frame', quat=True)
                self.create_frames(new_tvec, new_quat, 'marker_3', 'camera_color_optical_frame', quat=True)
        
        if self.marker_data[self.data_to_use]['name'] == 'grip':
          print("horizontal  = ", np.linalg.norm(id_4 - id_3))
          print("vertical  = ", np.linalg.norm(id_4 - id_7))

    return frame

  
  def create_frames(self, tvec, rvec, new_frame, ref_frame, quat=False):
    frame_pose = Pose()
    frame_pose.position.x = tvec[0]
    frame_pose.position.y = tvec[1]
    frame_pose.position.z = tvec[2]
    if not quat:
      quat = self.rodrigues_to_quaternion(rvec)
    else:
      quat = rvec
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
    
  def read_data_files(self, file_path):
    data = []
    with open(file_path) as f:
        for line in f.readlines():
            a_line = line.strip().split(',')
            a_line = list(map(lambda x:float(x), a_line))
            data.append(a_line)
    return data

  def get_transform(self, target_frame, source_frame):
    transformation_data = {
        "target_frame": String(target_frame),
        "source_frame": String(source_frame),
    }
    response = self.service_req(
        "/lookup_transform", LookupTransform, inputs=transformation_data
    )
    frame = response.frame_pose
    x = frame.position.x
    y = frame.position.y
    z = frame.position.z
    qx = frame.orientation.x
    qy = frame.orientation.y
    qz = frame.orientation.z
    qw = frame.orientation.w
    return np.array([x, y, z]), np.array([qx, qy, qz, qw])
  
  def calculate_hand_eye_transformation(self):
      tool_to_base = self.read_data_files('/home/bass/tool_to_base_transformations.txt')
      cam_to_target = self.read_data_files('/home/bass/target_to_cam_transformations.txt')
      R_gripper2base = []
      t_gripper2base = []
      for val in tool_to_base:
        t_gripper2base.append(np.array(val[:3]))
        quat = val[3:]
        mat = quaternion_matrix(quat)[:3,:3]
        R_gripper2base.append(mat)
      t_gripper2base = np.array(t_gripper2base)
      R_gripper2base = np.array(R_gripper2base)
      R_target2cam = []
      t_target2cam = []
      for val in cam_to_target:
        quat = val[3:]
        trans = np.array(val[:3])
        mat = quaternion_matrix(quat)[:3,:3]
        # hom_mat = np.eye(4)
        # hom_mat[:3,:3] = mat
        # hom_mat[:3,-1] = val[:3]
        # hom_mat = np.linalg.inv(hom_mat)
        # mat = hom_mat[:3,:3]
        # trans = hom_mat[:3,-1]
        # print(hom_mat)
        R_target2cam.append(mat)
        t_target2cam.append(trans)
      # n = np.random.random_integers(0, len(R_gripper2base)-1,4)
      n = np.arange(start=0, stop=len(R_gripper2base), step=1)
      # print(n)
      t_target2cam = np.array(t_target2cam)
      R_target2cam = np.array(R_target2cam)
      R_target2gripper, t_target2gripper = cv2.calibrateHandEye(R_gripper2base[n], t_gripper2base[n], R_target2cam[n], t_target2cam[n], method=cv2.CALIB_HAND_EYE_TSAI)
      # print(R_target2gripper)
      # print(t_target2gripper)
      rot_mat = np.eye(4)
      rot_mat[:3,:3] = R_target2gripper
      quat = quaternion_from_matrix(rot_mat).flatten()
      tvec = t_target2gripper.flatten()
      # tvec = np.dot(-R_target2gripper.T, t_target2gripper).flatten()
      # print("tvec = ", tvec)
      # print("quat = ", quat)
      self.create_frames(tvec, quat, 'estimated_marker', 'gripper_tip_link', quat=True)

  
  
    


if __name__ == '__main__':
  reader = CameraInfoReader()
  # reader.calculate_hand_eye_transformation()
  rospy.spin()