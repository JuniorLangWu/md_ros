#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
from sensor_msgs.msg import Image
import cv2
import mediapipe as mp
import message_filters
class Md_Dect:
    def __init__(self):

        # load parameters
        image_topic = rospy.get_param(
            '~image_topic', '/camera/color/image_raw')
        depth_topic = rospy.get_param(
            '~depth_topic', '/camera/aligned_depth_to_color/image_raw')
        self.visualize = rospy.get_param('~visualize', 'True')
        self.mp_drawing = mp.solutions.drawing_utils  # 画图
        self.mp_hands = mp.solutions.hands  # 检测人的双手

        self.hands = self.mp_hands.Hands(static_image_mode=False,
                                    max_num_hands=1,
                                    min_detection_confidence=0.5,
                                    min_tracking_confidence=0.5)# 模式参数设置
        self.frame = Image()
        self.depth = Image()
        self.getImageStatus = False

        # image, depth subscribe
        image = message_filters.Subscriber(image_topic, Image, queue_size=1, buff_size=52428800)
        depth = message_filters.Subscriber(depth_topic, Image, queue_size=1, buff_size=52428800)
        self.image_all = message_filters.TimeSynchronizer([image, depth], 1)
        self.image_all.registerCallback(self.callback)

        self.start = 0
        self.end = 0
        self.hand_points = []
        self.k_inv = np.array([[0.0011, 0, -0.6974], [0, 0.0011, -0.3893], [0, 0, 1]])
        self.median_depth = 500
        # if no image messages
        while (not self.getImageStatus):
            rospy.loginfo("waiting for image.")
            rospy.sleep(2)

    def callback(self, image, depth):
        # 处理时间
        self.end = rospy.Time.now().to_sec()
        rospy.loginfo("处理毫秒：%d", int((self.end-self.start)*1000))
        rospy.loginfo("每秒帧数：%d", int(1/(self.end-self.start)))
        self.start = rospy.Time.now().to_sec()

        self.getImageStatus = True
        self.frame = np.frombuffer(image.data, dtype=np.uint8).reshape(
            image.height, image.width, -1)
        #编码方式为16UC1
        self.depth = np.frombuffer(depth.data, dtype=np.uint16).reshape(
            depth.height, depth.width, -1)
        #中值滤波
        # self.depth = cv2.medianBlur(self.depth, 5)

        #输出图像尺寸
        # print(np.array(self.frame).shape)
        # print(np.array(self.depth).shape)
        # 输出深度
        # print("最大深度/mm:%d", self.depth.max())
        # print("最小深度/mm:%d", self.depth.min())

        # self.frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)#kinect使用
        #输入RGB
        #镜像
        self.frame = cv2.flip(self.frame, 1)
        self.depth = cv2.flip(self.depth, 1)
        #手部姿态估计
        self.results_hand = self.hands.process(self.frame)
        self.frame = cv2.cvtColor(self.frame, cv2.COLOR_RGB2BGR)
        self.dectshow(self.results_hand, image.height, image.width)
        # cv2.waitKey(1)

    def dectshow(self, hand, height,width):
        self.results_hand = hand
        if self.results_hand.multi_hand_landmarks:
            for self.hand_landmarks in self.results_hand.multi_hand_landmarks:
                self.mp_drawing.draw_landmarks(self.frame, self.hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                #连接拇指和食指指尖
                self.hand_points = []
                for id, hl in enumerate(self.hand_landmarks.landmark):
                    #输出地标点
                    # print('hand_landmarks:', hl)
                    # print('###' * 5)
                    cx, cy, cz = int(hl.x * width), int(hl.y * height), int(hl.z * width)
                    self.hand_points.append([cx, cy, cz])
                # cv2.line(self.frame, (self.hand_points[4][0], self.hand_points[4][1]),
                #          (self.hand_points[8][0], self.hand_points[8][1]), (0, 0, 255), 5)

                #提取坐标点深度和其中位数
                world_depth_points = []
                for x in range(21):
                    if (0<=self.hand_points[x][1]<720)&(0<=self.hand_points[x][0]<1280):
                        # print(x, ':', self.depth[self.hand_points[x][1], self.hand_points[x][0]])
                        world_depth_points.append(self.depth[self.hand_points[x][1], self.hand_points[x][0]])
                self.median_depth = np.median(world_depth_points)
                print(self.median_depth)
                #x，y像素坐标
                hand_xy_point = np.array(self.hand_points)[:, 0:2]

                #内参矩阵求逆计算世界坐标x，y
                hand_world_point = np.array([self.k_inv @ np.array([hand_xy_point[x, 0], hand_xy_point[x, 1], 1])
                                             for x in range(21)]) * self.median_depth
                # 计算到腕距离
                hand_zeros_point = np.array([[hand_world_point[i][0] - hand_world_point[0][0],
                                              hand_world_point[i][1] - hand_world_point[0][1]]
                                    for i in range(21)])
                # print(hand_zeros_point)
                # print(hand_zeros_point[12, 0], hand_zeros_point[12, 1])

                print('***' * 5)

        print('==='*10)
        if self.visualize:
            cv2.imshow('MediaPipe Poses', self.frame)
            # uint16_img = self.depth
            # uint16_img -= uint16_img.min()
            # uint16_img = uint16_img / (uint16_img.max() - uint16_img.min())
            # uint16_img *= 255
            # new_uint16_img = uint16_img.astype(np.uint8)
            # cv2.imshow('Depth', new_uint16_img)
            if cv2.waitKey(1) & 0xFF == 27:
                pass

def main():
    rospy.init_node('hand_ros', anonymous=True)
    md_dect = Md_Dect()
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass