#! /usr/bin/env python3

import roslib
import rospy
from std_msgs.msg import Header
from std_msgs.msg import String
from geometry_msgs.msg import Point
from rs_yolo.msg import Info

import random
from utils.torch_utils import select_device, load_classifier, time_sync
from utils.general import (
    check_img_size, non_max_suppression, apply_classifier, scale_coords,
    xyxy2xywh, strip_optimizer, set_logging)
from utils.datasets import LoadStreams, LoadImages, letterbox
from models.experimental import attempt_load
import torch.backends.cudnn as cudnn
import torch

import pyrealsense2 as rs
import math
import yaml
import argparse
import os
import time
import numpy as np
import sys
sys.path.remove('/opt/ros/noetic/lib/python2.7/dist-packages')

import cv2
from collections import deque

if __name__ == "__main__":
    rospy.init_node('ros_yolo')
    pub = rospy.Publisher("/detect_result_out", bbox, queue_size=10)

    print("[INFO] YoloV5目标检测-程序启动")
    print("[INFO] 开始YoloV5模型加载")
    # YOLOV5模型配置文件(YAML格式)的路径 yolov5_yaml_path
    # model = YoloV5(yolov5_yaml_path='/home/wdy/Yolov5_D435i_ws/src/rs_yolo/scripts/config/yolov5s.yaml')
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        print("[INFO] 完成YoloV5模型加载")
        rate.sleep()
        