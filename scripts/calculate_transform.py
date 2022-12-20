#!/usr/bin/env python
import sys
import rospy
import tf
import tf2_ros
from tf.transformations import quaternion_matrix, quaternion_from_matrix, translation_from_matrix
sys.path.append('/home/zaferpc/OpenSfM/')
from opensfm import transformations as tp
import yaml
from geometry_msgs.msg import PoseArray, Pose, TransformStamped, PoseStamped
import numpy as np
from math import fabs


class TransformCalulator:
    
    def __init__(self):
        not_transformed = False
        txt = True
        # rospy.Subscriber("/px_to_xyz", PoseArray, self.calcTransform)
        # self.ground_truth_pub = rospy.Publisher("/px_to_xyz", PoseArray, queue_size=1)
        # self.ground_truth_pub.publish(ground_truth_pose_arr)

        rospy.sleep(2)
        self.transformer_listener = tf.TransformListener()
        self.transformer_broadcaster = tf2_ros.StaticTransformBroadcaster()
        ground_truth_list = [[], [], []]
        if not txt:
            if not_transformed:
                with open("/home/zaferpc/abb_ws/src/abb_experimental/abb_irb120_moveit_config/launch/chess_corners_gt.yaml") as f:
                    self.ground_truth = yaml.load(f)
                self.transform_from_link_6_to_tool(self.ground_truth)
                print("PLEASE PASTE THE PRINTED POSES IN ground_truth.yaml, THEN PRESS ENTER")
                raw_input()
            with open("/home/zaferpc/abb_ws/src/abb_experimental/abb_irb120_moveit_config/launch/ground_truth.yaml") as f:
                self.ground_truth = yaml.load(f)
                
            for i, pose in enumerate(self.ground_truth['poses']):
                ground_truth_list[0].append(pose['position']['x'])
                ground_truth_list[1].append(pose['position']['y'])
                ground_truth_list[2].append(pose['position']['z'])
        else:
            with open('/home/bass/ur5_rob_ws/src/calibration/rob.txt') as f:
                for line in f.readlines():
                    a_line = line.strip().split(',')
                    a_line = list(map(lambda x:float(x), a_line))
                    ground_truth_list[0].append(a_line[0])
                    ground_truth_list[1].append(a_line[1])
                    ground_truth_list[2].append(a_line[2])
        
        self.ground_truth_arr = np.array(ground_truth_list)             

        # ground_truth_pose_arr = self.convert_pose_yaml_to_pose_array(self.ground_truth)
        
    def calcTransform(self, detected_msg):
        
        # calibrate or measure accuracy
        print("recalibrate or measure accuracy ? 1: recalibrate, else: measure accuracy")
        x = input()
        if x == '1':
            # capture the samples
            detected_list = [[], [], []]
            # for i, pose in enumerate(detected_msg.poses):
            #     detected_list[0].append(pose.position.x)
            #     detected_list[1].append(pose.position.y)
            #     detected_list[2].append(pose.position.z)
            with open('/home/bass/ur5_rob_ws/src/calibration/cam.txt') as f:
                for line in f.readlines():
                    a_line = line.strip().split(',')
                    a_line = list(map(lambda x:float(x), a_line))
                    detected_list[0].append(a_line[0])
                    detected_list[1].append(a_line[1])
                    detected_list[2].append(a_line[2])
            detected_arr = np.array(detected_list)
            
            # calculate the transformation of the camera wrt the base_link
            transform = tp.affine_matrix_from_points(detected_arr, self.ground_truth_arr, False, False, usesvd=False)
            
            # broadcast this transformation
            translation = translation_from_matrix(transform)
            orientation = quaternion_from_matrix(transform)
            
            static_transformStamped = TransformStamped()
            static_transformStamped.header.stamp = rospy.Time.now()
            static_transformStamped.header.frame_id = "base_link"
            static_transformStamped.child_frame_id = "calibrated_frame"
            static_transformStamped.transform.translation.x = translation[0]
            static_transformStamped.transform.translation.y = translation[1]
            static_transformStamped.transform.translation.z = translation[2]
            static_transformStamped.transform.rotation.x = orientation[0]
            static_transformStamped.transform.rotation.y = orientation[1]
            static_transformStamped.transform.rotation.z = orientation[2]
            static_transformStamped.transform.rotation.w = orientation[3]
            self.transformer_broadcaster.sendTransform(static_transformStamped)
            
            # get the transformation of the camera wrt link_6
            # self.transformer_listener.waitForTransform(
            #     "tool0", "calibrated_frame",  rospy.Time(), rospy.Duration(1))
            # translation, orientation = self.transformer_listener.lookupTransform(
            #     "tool0", "calibrated_frame", rospy.Time())
                        
            # # broadcast the transformation
            # static_transformStamped = TransformStamped()
            # static_transformStamped.header.stamp = rospy.Time.now()
            # static_transformStamped.header.frame_id = "tool0"
            # static_transformStamped.child_frame_id = "calibrated_frame"
            # static_transformStamped.transform.translation.x = translation[0]
            # static_transformStamped.transform.translation.y = translation[1]
            # static_transformStamped.transform.translation.z = translation[2]
            # static_transformStamped.transform.rotation.x = orientation[0]
            # static_transformStamped.transform.rotation.y = orientation[1]
            # static_transformStamped.transform.rotation.z = orientation[2]
            # static_transformStamped.transform.rotation.w = orientation[3]
            # self.transformer_broadcaster.sendTransform(static_transformStamped)
            
            print(translation)
            print(orientation)
            
        else:
            # transform the captured samples to the base_link
            detected_transformed = self.__transform_poses("base_link", "calibrated_frame", detected_msg)
            
            error = [[], [], []]
            # compare the transformed detected poses to the ground truth
            for i, pose in enumerate(detected_transformed.poses):
                error[0].append(fabs(pose.position.x - self.ground_truth_arr[0, i]))
                error[1].append(fabs(pose.position.y - self.ground_truth_arr[1, i]))
                error[2].append(fabs(pose.position.z - self.ground_truth_arr[2, i]))

            error_arr = np.array(error)
            error_matrix = np.zeros((3, 3))
            # x errors
            error_matrix[0, 0] = np.max(error_arr[0, :]) 
            error_matrix[0, 1] = np.min(error_arr[0, :])
            error_matrix[0, 2] = np.mean(error_arr[0, :])
            
            # y errors
            error_matrix[1, 0] = np.max(error_arr[1, :])
            error_matrix[1, 1] = np.min(error_arr[1, :])
            error_matrix[1, 2] = np.mean(error_arr[1, :])
            
            # z errors
            error_matrix[2, 0] = np.max(error_arr[2, :])
            error_matrix[2, 1] = np.min(error_arr[2, :])
            error_matrix[2, 2] = np.mean(error_arr[2, :])
            
            print(error_matrix)

    def __transform_poses(self, target_frame, source_frame, pose_arr):
        print ("here")
        trans_pose_arr = PoseArray()
        for i in range(len(pose_arr.poses)):
            trans_pose = PoseStamped()
            pose = PoseStamped()
            pose.header.frame_id = source_frame
            pose.pose = pose_arr.poses[i]
            self.transformer_listener.waitForTransform(target_frame, source_frame, rospy.Time(),rospy.Duration(1))
            trans_pose = self.transformer_listener.transformPose(target_frame, pose)
            trans_pose_arr.poses.append(trans_pose.pose)
            
        trans_pose_arr.header.frame_id = target_frame
        trans_pose_arr.header.stamp = rospy.Time()    
        return trans_pose_arr
    
    def transform_from_link_6_to_tool(self, pose_yaml):
        self.transformer_listener.waitForTransform(
            "link_6", "milling_tool",  rospy.Time(), rospy.Duration(1))
        t, r = self.transformer_listener.lookupTransform(
            "link_6", "milling_tool", rospy.Time())
        T_m_6 = quaternion_matrix(r)
        T_m_6[:-1, -1] = t
        transformed_pose_array = PoseArray()
        for pose in pose_yaml['poses']:
            x = pose['position']['x']
            y = pose['position']['y']
            z = pose['position']['z']
            qx = pose['orientation']['x']
            qy = pose['orientation']['y']
            qz = pose['orientation']['z']
            qw = pose['orientation']['w']
            T_6_b = quaternion_matrix([qx, qy, qz, qw])
            T_6_b[:-1, -1] = [x, y, z]
            trans_arr = T_6_b.dot(T_m_6)
            quat = quaternion_from_matrix(trans_arr)
            translation = translation_from_matrix(trans_arr) 
            new_pose = Pose()
            new_pose.position.x = translation[0]
            new_pose.position.y = translation[1]
            new_pose.position.z = translation[2]
            new_pose.orientation.x = quat[0]
            new_pose.orientation.y = quat[1]
            new_pose.orientation.z = quat[2]
            new_pose.orientation.w = quat[3]
            transformed_pose_array.poses.append(new_pose)
        print(transformed_pose_array)
        return transformed_pose_array
    
    def convert_pose_yaml_to_pose_array(self, pose_yaml):
        pose_array = PoseArray()
        for pose in pose_yaml['poses']:
            new_pose = Pose()
            new_pose.position.x = pose['position']['x']
            new_pose.position.y = pose['position']['y']
            new_pose.position.z = pose['position']['z']
            new_pose.orientation.x = pose['orientation']['x']
            new_pose.orientation.y = pose['orientation']['y']
            new_pose.orientation.z = pose['orientation']['z']
            new_pose.orientation.w = pose['orientation']['w']
            pose_array.poses.append(new_pose)
        return pose_array

            
if __name__ == "__main__":
    rospy.init_node("transform_calculator")
    tc = TransformCalulator()
    tc.calcTransform(None)
    # rospy.spin()