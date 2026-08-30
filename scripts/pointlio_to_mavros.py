#!/usr/bin/python3
#上面的python3不一定是这样写，建议找ai优化一下代码，和前面的推荐文章里的CPP代码其实一样的，该代码发布频率不太对
 
 
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import tf
import numpy as np
from collections import deque
import math
 
# 滑动窗口平均类，用于平滑 yaw 值
class SlidingWindowAverage:
    def __init__(self, window_size):
        self.window_size = window_size
        self.data_queue = deque()
        self.window_sum = 0.0
 
    def add_data(self, new_data):
        # 如果新数据与上一个数据差异过大，重置队列
        if self.data_queue and abs(new_data - self.data_queue[-1]) > 0.01:
            self.data_queue.clear()
            self.window_sum = 0.0
        self.data_queue.append(new_data)
        self.window_sum += new_data
 
        # 如果队列大小超过窗口大小，移除最早的数据
        if len(self.data_queue) > self.window_size:
            self.window_sum -= self.data_queue.popleft()
        return self.window_sum / len(self.data_queue)
 
    def get_size(self):
        return len(self.data_queue)
 
    def get_avg(self):
        if self.data_queue:
            return self.window_sum / len(self.data_queue)
        else:
            return 0.0
 
class FastLIOToMavros:
    def __init__(self):
        rospy.init_node('pointlio_to_mavros', anonymous=True)
 
        # 初始化位姿和四元数
        self.p_lidar_body = np.zeros(3)
        self.q_mav = [0, 0, 0, 1]
        self.q_px4_odom = [0, 0, 0, 1]
 
        self.window_size = 8
        self.swa = SlidingWindowAverage(self.window_size)
 
        self.init_flag = False
        self.init_q = tf.transformations.quaternion_from_euler(0, 0, 0)
 
        # 订阅 Fast-LIO 的 Odometry 数据
        rospy.Subscriber('/pointlio/odom', Odometry, self.vins_callback)
        # 订阅 PX4 的本地位置 Odometry 数据
        rospy.Subscriber('/mavros/odometry/in', Odometry, self.px4_odom_callback)
        # 发布视觉位姿数据到 PX4
        self.vision_pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=10)
 
        self.rate = rospy.Rate(30.0)
        self.run()
 
    def from_quaternion_to_yaw(self, q):
        # 将四元数转换为 yaw 角
        euler = tf.transformations.euler_from_quaternion(q)
        return euler[2]
 
    def vins_callback(self, msg):
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
        yaw = self.from_quaternion_to_yaw(self.q_px4_odom)
        self.swa.add_data(yaw)
 
    def run(self):
        while not rospy.is_shutdown():
            rospy.loginfo_throttle(1, f"Current Window Size: {self.swa.get_size()}")
            # 初始化 yaw 角
            if self.swa.get_size() == self.window_size and not self.init_flag:
                init_yaw = self.swa.get_avg()
                self.init_q = tf.transformations.quaternion_from_euler(0, 0, init_yaw)
                self.init_flag = True
 
            if self.init_flag:
                # 旋转位姿以对齐初始 yaw 角
                rot_matrix = tf.transformations.quaternion_matrix(self.init_q)[:3, :3]
                p_enu = np.dot(rot_matrix, self.p_lidar_body)
 
                # 构建并发布视觉位姿消息
                vision = PoseStamped()
                vision.header.stamp = rospy.Time.now()
                vision.header.frame_id = "map"  # 根据实际情况设置
 
                vision.pose.position.x = p_enu[0]
                vision.pose.position.y = p_enu[1]
                vision.pose.position.z = p_enu[2]
 
                vision.pose.orientation.x = self.q_mav[0]
                vision.pose.orientation.y = self.q_mav[1]
                vision.pose.orientation.z = self.q_mav[2]
                vision.pose.orientation.w = self.q_mav[3]
 
                self.vision_pub.publish(vision)
 
                rospy.loginfo(
                    "\nPosition in ENU:\n   x: {:.3f}\n   y: {:.3f}\n   z: {:.3f}\nOrientation of LiDAR:\n   x: {:.3f}\n   y: {:.3f}\n   z: {:.3f}\n   w: {:.3f}".format(
                        p_enu[0], p_enu[1], p_enu[2],
                        self.q_mav[0], self.q_mav[1], self.q_mav[2], self.q_mav[3]
                    )
                )
 
            self.rate.sleep()
 
if __name__ == '__main__':
    try:
        FastLIOToMavros()
    except rospy.ROSInterruptException:
        pass