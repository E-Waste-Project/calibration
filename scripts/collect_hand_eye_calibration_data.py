#!/usr/bin/env python

import rospy
from robot_helpers.srv import LookupTransform
from os import system, path, getcwd
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import argparse


class HandEyeCalibrationDataCollector:
  def __init__(self, rob_base_frame, cam_frame, target_frame, tool_frame, save_path, remove_first):
    """In a general sense, this class finds a transfomration that is static (like hand-eye transfomration if eye is mounted on hand)
    given many unique instances of two dynamic transformations, one is to the camera, and other to the tool, to find camera to tool transformations"""
    rospy.init_node('hand_eye_calibration_data_collector')
    # remove_first = input('remove files first? y/n \n')
    self.tool_to_base_transformations_txt = path.join(save_path,'tool_to_base_transformations.txt')
    self.cam_to_target_transformations_txt = path.join(save_path,'cam_to_target_transformations.txt')
    self.joint_states_txt = path.join(save_path,'joint_states.txt')
    if remove_first:
        system('rm {}'.format(self.tool_to_base_transformations_txt))
        system('rm {}'.format(self.cam_to_target_transformations_txt))
        system('rm {}'.format(self.joint_states_txt))
    self.tool_to_base_transformations = []
    self.cam_to_target_transformations = []
    self.joint_states = []
    self.rob_base_frame = rob_base_frame
    self.cam_frame = cam_frame
    self.target_frame = target_frame
    self.tool_frame = tool_frame
    rospy.Subscriber('/collect_data_request', String, self.collect_transformations)
    
  def collect_transformations(self, msg):
    transformation_data = {
        "target_frame": String(self.cam_frame),
        "source_frame": String(self.target_frame),
    }
    response = self.service_req(
        "/lookup_transform", LookupTransform, inputs=transformation_data
    )
    cam_to_target = response.frame_pose
    self.cam_to_target_transformations.append([cam_to_target.position.x,
                            cam_to_target.position.y,
                            cam_to_target.position.z,
                            cam_to_target.orientation.x,
                            cam_to_target.orientation.y,
                            cam_to_target.orientation.z,
                            cam_to_target.orientation.w,])
    transformation_data = {
        "target_frame": String(self.tool_frame),
        "source_frame": String(self.rob_base_frame),
    }
    response = self.service_req(
        "/lookup_transform", LookupTransform, inputs=transformation_data
    )
    tool_to_base = response.frame_pose
    self.tool_to_base_transformations.append([tool_to_base.position.x,
                            tool_to_base.position.y,
                            tool_to_base.position.z,
                            tool_to_base.orientation.x,
                            tool_to_base.orientation.y,
                            tool_to_base.orientation.z,
                            tool_to_base.orientation.w,])
    joints_msg = rospy.wait_for_message("/joint_states",JointState)
    self.joint_states.append(joints_msg.position)
    self.append_new_line()
    print("saved transformations")
  
  def service_req(self, name, service_type, **inputs):
    _ = rospy.wait_for_service(name)
    try:
        callable_service_func = rospy.ServiceProxy(name, service_type)
        response = callable_service_func(**inputs["inputs"])
        return response
    except rospy.ServiceException as e:
        print("Service Failed : {}".format(e))
  
  def append_new_line(self):
    """Append given text as a new line at the end of file"""
    # Open the file in append & read mode ('a+')
    with open(self.tool_to_base_transformations_txt, "a+") as file_object:
        # Move read cursor to the start of file.
        file_object.seek(0)
        # If file is not empty then append '\n'
        data = file_object.read(100)
        if len(data) > 0:
            file_object.write("\n")
        # Append text at the end of file
        file_object.write(','.join(str(j) for j in self.tool_to_base_transformations[-1]))
    with open(self.cam_to_target_transformations_txt, "a+") as file_object:
        # Move read cursor to the start of file.
        file_object.seek(0)
        # If file is not empty then append '\n'
        data = file_object.read(100)
        if len(data) > 0:
            file_object.write("\n")
        # Append text at the end of file
        file_object.write(','.join(str(j) for j in self.cam_to_target_transformations[-1]))
    with open(self.joint_states_txt, "a+") as file_object:
        # Move read cursor to the start of file.
        file_object.seek(0)
        # If file is not empty then append '\n'
        data = file_object.read(100)
        if len(data) > 0:
            file_object.write("\n")
        # Append text at the end of file
        file_object.write(','.join(str(j) for j in self.joint_states[-1]))
        
  
if __name__ == '__main__':
    args = argparse.ArgumentParser(description='Test the rosprolog service')
    args.add_argument('-r', '--remove_first', action='store_true', help='remove files first')
    args.add_argument('-p', '--save_path', type=str, help='save path', default=getcwd())
    args.add_argument('-t', '--target_frame', type=str, help='name of detected target frame', default='marker_3')
    args.add_argument('-c', '--camera_frame', type=str, help='name of camera frame', default='camera_color_optical_frame')
    args.add_argument('-b', '--base_frame', type=str, help='name of base frame', default='base_link')
    args.add_argument('-e', '--end_effector_frame', type=str, help='name of end effector frame', default='gripper_tip_link')
    remove_first = args.parse_args().remove_first
    save_path = args.parse_args().save_path
    target_frame = args.parse_args().target_frame
    camera_frame = args.parse_args().camera_frame
    base_frame = args.parse_args().base_frame
    tool_frame = args.parse_args().end_effector_frame
    HandEyeCalibrationDataCollector(base_frame, camera_frame, target_frame, tool_frame, save_path, remove_first)
    rospy.sleep(1)
    rospy.spin()