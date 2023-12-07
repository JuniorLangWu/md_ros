#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
import torch
import rospy
import numpy as np
from time import time
from std_msgs.msg import Header
from sensor_msgs.msg import Image
import cv2
import mediapipe as mp
class Md_Dect:
    def __init__(self):

        # load parameters
        image_topic = rospy.get_param(
            '~image_topic', '/camera/color/image_raw')
        depth_topic = rospy.get_param(
            '~depth_topic', '/camera/aligned_depth_to_color/image_raw')
        self.visualize = rospy.get_param('~visualize', 'True')
        self.mp_drawing = mp.solutions.drawing_utils  # 画图
        self.mp_poses = mp.solutions.pose  # 检测人的姿势
        self.mp_hands = mp.solutions.hands  # 检测人的双手
        self.poses = self.mp_poses.Pose( static_image_mode=False,
                                    min_detection_confidence=0.5,
                                    min_tracking_confidence=0.5)  # 模式参数设置

        self.hands = self.mp_hands.Hands(static_image_mode=False,
                                    max_num_hands=2,
                                    min_detection_confidence=0.5,
                                    min_tracking_confidence=0.5)
        self.frame = Image()
        self.getImageStatus = False

        # image subscribe
        self.color_sub = rospy.Subscriber(image_topic, Image, self.image_callback,
                                          queue_size=1, buff_size=52428800)

        # if no image messages
        while (not self.getImageStatus):
            rospy.loginfo("waiting for image.")
            rospy.sleep(2)

    def image_callback(self, image):
        self.getImageStatus = True
        self.frame = np.frombuffer(image.data, dtype=np.uint8).reshape(
            image.height, image.width, -1)

        # self.frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)#kineck使用

        self.frame = cv2.flip(self.frame, 1)

        self.results_pose = self.poses.process(self.frame)
        self.results_hand = self.hands.process(self.frame)

        self.frame = cv2.cvtColor(self.frame, cv2.COLOR_RGB2BGR)
        self.dectshow(self.results_pose, self.results_hand, image.height, image.width)

        cv2.waitKey(3)

    def dectshow(self, pose, hand, height, width):
        self.results_pose = pose
        self.results_hand = hand
        if self.results_pose.pose_landmarks:
            for pose_landmarks in self.results_pose.pose_landmarks.landmark:
                print('pose_landmarks:', pose_landmarks)
                print('###' * 5)
                self.mp_drawing.draw_landmarks(
                    self.frame, self.results_pose.pose_landmarks, self.mp_poses.POSE_CONNECTIONS)
        print('---' * 10)
        if self.results_hand.multi_hand_landmarks:
            for self.hand_landmarks in self.results_hand.multi_hand_landmarks:
                self.mp_drawing.draw_landmarks(self.frame, self.hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                hand_points = []
                for id, hl in enumerate(self.hand_landmarks.landmark):
                    print('hand_landmarks:', hl)
                    print('###' * 5)
                    cx, cy = int(hl.x * width), int(hl.y * height)
                    hand_points.append([cx, cy])
                cv2.line(self.frame, (hand_points[4][0], hand_points[4][1]),
                         (hand_points[8][0], hand_points[8][1]), (0, 0, 255), 5)
                print('---' * 10)
        print('==='*10)
        if self.visualize:
            cv2.imshow('MediaPipe Poses', self.frame)
            if cv2.waitKey(1) & 0xFF == 27:
                pass
def main():
    rospy.init_node('md_ros', anonymous=True)
    md_dect = Md_Dect()
    rospy.spin()


if __name__ == "__main__":

    main()