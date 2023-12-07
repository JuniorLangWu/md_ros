import cv2
import torch
from time import time
from std_msgs.msg import Header
pass
#回调函数
self.color_sub = rospy.Subscriber(image_topic, Image, self.image_callback,
                                  queue_size=1, buff_size=52428800)
self.image_all = message_filters.ApproximateTimeSynchronizer([image, depth], 2, 0.01, allow_headerless=True)
self.hand_world_points = []
世界地标
if self.results_hand.multi_hand_world_landmarks:
    for self.hand_world_landmarks in self.results_hand.multi_hand_world_landmarks:
        self.hand_world_points = []
        for wid, whl in enumerate(self.hand_world_landmarks.landmark):
            #毫米化
            wx, wy, wz = whl.x * 1000, whl.y * 1000, whl.mean_z * 1000
            self.hand_world_points.append([wx, wy, wz])
# 显示测量拇指指尖和食指指尖长度
long = ((self.hand_world_points[8][0]-self.hand_world_points[4][0])**2
         +(self.hand_world_points[8][1]-self.hand_world_points[4][1])**2
         +(self.hand_world_points[8][2]-self.hand_world_points[4][2])**2)**0.5
print(self.hand_world_points[8][0]-self.hand_world_points[4][0])
print(self.hand_world_points[8][1]-self.hand_world_points[4][1])
print(self.hand_world_points[8][2]-self.hand_world_points[4][2])
print(long)
print(self.hand_world_points[0][0])
cv2.putText(self.frame, "long--{}".format(long),(10,450),cv2.FONT_HERSHEY_PLAIN,
            3,(0,0,255),3)
# # 输出世界地标原点及。。。
print(self.hand_world_points[9][0])
print(self.hand_world_points[9][1])
print(self.hand_world_points[9][2])
print(self.hand_world_points[12][0])
print(self.hand_world_points[12][1])
print(self.hand_world_points[12][2])
# 输出腕部像素地标
print(self.hand_points[0][0], self.hand_points[0][1], self.hand_points[0][2])

# #计算像素和world的的比例
hand_zeros = [self.hand_points[9][:] for num in range(21)]
hand_zeros_point = [[self.hand_points[i][0]-hand_zeros[i][0],
                     self.hand_points[i][1]-hand_zeros[i][1],
                     self.hand_points[i][2]-hand_zeros[i][2]]
                    for i in range(21)]
print(hand_zeros_point[9][:])
print(hand_zeros_point[12][:])
scale = [[hand_zeros_point[i][0]/self.hand_world_points[i][0],
          hand_zeros_point[i][1]/self.hand_world_points[i][1],
          hand_zeros_point[i][2]/self.hand_world_points[i][2]]
         for i in range(21)]
scale = [[int(hand_zeros_point[i][0]/self.hand_world_points[i][0]),
          int(hand_zeros_point[i][1]/self.hand_world_points[i][1]),
          int(hand_zeros_point[i][2]/self.hand_world_points[i][2])]
         for i in range(21)]
print(scale)

#RGB
self.RGB1 = Image()
self.RGB2 = Image()
self.RGB3 = Image()
self.RGB1 = self.frame[:, :, 0]
self.RGB2 = self.frame[:, :, 1]
self.RGB3 = self.frame[:, :, 2]