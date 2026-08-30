#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import cv2.aruco as aruco
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class ArUcoDetector:
    def __init__(self):
        # 1. 初始化 ROS 节点
        rospy.init_node('aruco_detector_node', anonymous=True)
        self.bridge = CvBridge()
        
        # 2. 兼容性配置 ArUco 字典和参数
        # 针对你图片中的 6x6_250 字典
        try:
            # 尝试旧版 OpenCV API (Noetic 默认)
            self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
            self.parameters = aruco.DetectorParameters_create()
            rospy.loginfo("使用 OpenCV 旧版 ArUco API")
        except AttributeError:
            # 尝试新版 OpenCV API (4.7+)
            self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
            self.parameters = aruco.DetectorParameters()
            rospy.loginfo("使用 OpenCV 新版 ArUco API")

        # 3. 话题配置
        # 订阅：请确保这里的 /usb_cam/image_raw 与你的相机驱动话题一致
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback)
        
        # 发布：发布识别标注后的图像
        self.image_pub = rospy.Publisher("/aruco/detection_result", Image, queue_size=1)
        
        rospy.loginfo("ArUco 识别节点已启动，等待图像输入...")

    def callback(self, data):
        try:
            # 将 ROS 图像转为 OpenCV 格式
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
            return

        # 4. ArUco 检测核心逻辑
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        # 5. 标注处理
        if ids is not None:
            # 在原图上画出绿色方框和 ID
            aruco.drawDetectedMarkers(frame, corners, ids, borderColor=(0, 0, 255))
            
            # 打印识别到的 ID 到终端
            for i in range(len(ids)):
                pts = np.int32(corners[i][0])
                cv2.polylines(frame, [pts], True, (0, 0, 255), thickness=10)
                id_val = ids[i][0]
                rospy.loginfo("OK成功 检测到标记 ID: {}".format(id_val))
                
                # 在画面上加一段酷炫的文字标注
                cv2.putText(frame, "TARGET ID: {}".format(id_val), 
                            (int(corners[i][0][0][0]), int(corners[i][0][0][1]) - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), thickness=5)
        else:
            # 如果没识别到，可以在画面角落显示状态
            cv2.putText(frame, "Searching...", (20, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        # 6. 发布结果图像
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
        except CvBridgeError as e:
            rospy.logerr(e)

if __name__ == '__main__':
    try:
        detector = ArUcoDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("正在关闭识别节点...")
        cv2.destroyAllWindows()