#ifndef ASNAV_API_H
#define ASNAV_API_H

#include "mavros_msgs/CommandBool.h"
#include <mavros_msgs/CommandLong.h>
#include "mavros_msgs/PositionTarget.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/State.h"
#include "nav_msgs/Odometry.h"
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>
#include <yolov8_ros_msgs/BoundingBoxes.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <string>

#include <move_base_msgs/MoveBaseAction.h> 
#include <actionlib/client/simple_action_client.h> 

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class ASNAV
{
    public:
    explicit ASNAV(ros::NodeHandle& nh_);
    ~ASNAV();
    bool takeoff(float height);
    bool position(float x, float y, float z, float yaw, float tol = 0.2f);
    bool positionSmooth(float target_x, float target_y, float target_z, float tol, float hover_sec = 0.0f);
    bool navigation(float x, float y, float z, float yaw, float tol = 0.2f);
    bool controlYaw(float x, float y, float z, float target_yaw, float wait_sec);
    bool flyDown(float descend_z);
    bool flyUp(float height);
    void setpointPublish();
    void set_mode(std::string mode);
    bool autoLand();
    bool trackYoloDown(float max_distance = 0.35f, int tol = 30);
    bool trackYoloForward(float Kp_x, float Kp_y, float Kp_z, float target_box_height, int tol_xy, int tol_size);
    bool trackYoloing(float Kp_x, float Kp_y, float Kp_z, float target_box_height, int tol_xy, int tol_size);
    void reset_target();
    bool interceptBalloon( float charge_speed, float Kp_y, float Kp_z, float pop_box_height);
    bool escapeBackward(float distance, float tol);
    bool attackBalloon(float charge_speed);
    bool pwmControl(int pwm_channel_5, int pwm_channel_6, int pwm_channel_7 = 0);

    struct yoloBox
    {
        std::string Class;
        float cameraXCenter, cameraYCenter;
        float boxHeight;
    };

    float fly_height;            
    float descend_z;
    
    private:
    //容差函数
    float tolerance(float x, float y, float z) const;
    //mavros回调
    void mavros_state_cb(const mavros_msgs::State::ConstPtr& msg);
    void mavros_local_position_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    //move_base回调
    void planner_cmd_vel_cb(const geometry_msgs::Twist::ConstPtr& msg);
    //yolo回调
    void yolo_info_cb(const yolov8_ros_msgs::BoundingBoxes::ConstPtr& msg);
    ros::NodeHandle nh_;
    //mavros相关组件
    ros::Subscriber mavros_state_sub_, mavros_local_position_pose_sub_;
    ros::Publisher mavros_setpoint_raw_local_pub_;
    ros::ServiceClient set_mode_client_;
    ros::ServiceClient mavros_cmd_command_client_;
    //move_base相关组件
    ros::Subscriber planner_cmd_vel_sub_;  
    ros::Publisher goal_pub_;
    //yolo相关组件
    ros::Subscriber yolo_info_sub_;
    //mavros变量
    bool is_offboard, is_auto_land, is_yaw_finished = false;
    geometry_msgs::Point current_position;
    mavros_msgs::PositionTarget target_position;
    mavros_msgs::State current_state;
    float current_yaw;
    ros::Time yaw_finish_time;

    // mavros_msgs::CommandLong lib_ctrl_pwm;

    //move_base变量
    geometry_msgs::Point planner_velxy_posz;
    MoveBaseClient* ac_;
    bool goal_sent_;
    //其他变量
    double start_planning_time, finish_planning_time;
    bool is_as_received = false;

    yoloBox yolo_box_info;
    std::string target_class_name;
    float integral_error_x, integral_error_y, last_err_x, last_err_y;
    ros::Time last_yolo_time_;
};
#endif