#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
from time import time
from sensor_msgs.msg import Image
import cv2
import mediapipe as mp
import message_filters
def main():

    # start = time()
    k_inv = np.array([[1.0993, 0, -697.3866],[0, 1.1004, -389.2588],[0, 0, 1000]])
    p = np.array([1, 1, 1])

    p[4] = 2
    print(p)
    # print([i for i in 0:2])
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass