#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import tf
import numpy as np
from collections import deque

# 滑动窗口平均类，用于平滑 yaw 值
class SlidingWindowAverage:
    def __init__(self, window_size):
        self.window_size = window_size
        self.data_queue = deque(maxlen=window_size)  # 使用固定大小的队列
        
    def add_data(self, new_data):
        # 如果队列非空且新数据与最新数据差异过大，重置队列
        if self.data_queue and abs(new_data - self.data_queue[-1]) > 0.01:
            self.data_queue.clear()
        
        self.data_queue.append(new_data)
        return self.get_avg()
    
    def get_size(self):
        return len(self.data_queue)
    
    def get_avg(self):
        if self.data_queue:
            return sum(self.data_queue) / len(self.data_queue)
        else:
            return 0.0

class FastLIOToMavros:
    def __init__(self):
        rospy.init_node('fastlio_to_mavros', anonymous=True)
        
        # 获取参数
        self.window_size = rospy.get_param('~window_size', 8)
        publish_rate = rospy.get_param('~publish_rate', 30.0)
        
        # 初始化位姿和四元数
        self.p_lidar_body = np.zeros(3)
        self.q_mav = [0, 0, 0, 1]
        self.q_px4_odom = [0, 0, 0, 1]
        
        # 创建滑动窗口平均器
        self.swa = SlidingWindowAverage(self.window_size)
        
        # 初始化标志
        self.init_flag = False
        self.init_q = tf.transformations.quaternion_from_euler(0, 0, 0)
        
        # 订阅和发布器
        self.vins_sub = rospy.Subscriber('~fastlio_odom', Odometry, self.fastlio_callback, queue_size=10)
        self.px4_odom_sub = rospy.Subscriber('~px4_odom', Odometry, self.px4_odom_callback, queue_size=10)
        self.vision_pub = rospy.Publisher('~vision_pose', PoseStamped, queue_size=10)
        
        # 定时器控制发布频率
        self.timer = rospy.Timer(rospy.Duration(1.0/publish_rate), self.publish_vision_pose)
        
        rospy.loginfo("FastLIO to MAVROS converter initialized")
        rospy.loginfo(f"Window size: {self.window_size}, Publish rate: {publish_rate} Hz")
        
    def from_quaternion_to_yaw(self, q):
        # 将四元数转换为 yaw 角
        euler = tf.transformations.euler_from_quaternion(q)
        return euler[2]
    
    def fastlio_callback(self, msg):
        # 获取 Fast-LIO 提供的位姿和四元数
        self.p_lidar_body = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        self.q_mav = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]
    
    def px4_odom_callback(self, msg):
        # 获取 PX4 的本地位置四元数，并计算 yaw 角
        self.q_px4_odom = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]
        
        # 更新滑动窗口平均
        yaw = self.from_quaternion_to_yaw(self.q_px4_odom)
        self.swa.add_data(yaw)
        
        # 初始化 yaw 角
        if self.swa.get_size() == self.window_size and not self.init_flag:
            init_yaw = self.swa.get_avg()
            self.init_q = tf.transformations.quaternion_from_euler(0, 0, init_yaw)
            self.init_flag = True
            rospy.loginfo(f"Initial yaw initialized: {init_yaw:.3f} rad")
    
    def publish_vision_pose(self, event):
        if not self.init_flag:
            return
        
        # 旋转位姿以对齐初始 yaw 角
        rot_matrix = tf.transformations.quaternion_matrix(self.init_q)[:3, :3]
        p_enu = np.dot(rot_matrix, self.p_lidar_body)
        
        # 构建视觉位姿消息
        vision_msg = PoseStamped()
        vision_msg.header.stamp = rospy.Time.now()
        vision_msg.header.frame_id = "map"  # 根据实际情况设置
        
        vision_msg.pose.position.x = p_enu[0]
        vision_msg.pose.position.y = p_enu[1]
        vision_msg.pose.position.z = p_enu[2]
        
        vision_msg.pose.orientation.x = self.q_mav[0]
        vision_msg.pose.orientation.y = self.q_mav[1]
        vision_msg.pose.orientation.z = self.q_mav[2]
        vision_msg.pose.orientation.w = self.q_mav[3]
        
        # 发布消息
        self.vision_pub.publish(vision_msg)
        
        # 每100次发布打印一次信息（减少终端输出）
        if rospy.get_time() % 10 < 0.01:
            rospy.loginfo_throttle(1.0, 
                f"Position (ENU): x={p_enu[0]:.3f}, y={p_enu[1]:.3f}, z={p_enu[2]:.3f}"
            )

def main():
    try:
        converter = FastLIOToMavros()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("FastLIO to MAVROS converter shutting down")
        pass

if __name__ == '__main__':
    main()