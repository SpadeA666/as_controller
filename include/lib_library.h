#ifndef __LIB_LIBRARY__
#define __LIB_LIBRARY__

#include <string> 
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>


#include <std_msgs/Bool.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <cmath>

#include <nav_msgs/Odometry.h>
#include <mavros_msgs/CommandLong.h>   
#include <string>
#include <geometry_msgs/Twist.h>

 
using namespace std;
//1、无人机状态回调函数
//1、定义变量，用于接收无人机的状态信息
mavros_msgs::State lib_current_state;
void lib_state_cb(const mavros_msgs::State::ConstPtr& msg);


//2、定义变量，用于接收无人机的里程计信息,定义存储r\p\y的变量，定义四元素变量
tf::Quaternion lib_quat; 
nav_msgs::Odometry lib_local_pos; 
double lib_roll, lib_pitch, lib_yaw;
float lib_init_position_x_take_off =0;
float lib_init_position_y_take_off =0;
float lib_init_position_z_take_off =0;
bool  lib_flag_init_position = false;
void  lib_lib_local_pos_cb(const nav_msgs::Odometry::ConstPtr& msg);


//3、延时函数接口，单位秒
ros::Time lib_mission_success_time_record;
bool lib_time_record_start_flag = false;
bool lib_time_record_func(float time_duration, ros::Time time_now);

//舵机相关
//4、控制舵机函数接口，默认接辅助通道5和6，占空同比设置0-100，代表控制舵机的不同角度
mavros_msgs::CommandLong lib_ctrl_pwm;
void lib_pwm_control(int pwm_channel_5, int pwm_channel_6);


#endif
