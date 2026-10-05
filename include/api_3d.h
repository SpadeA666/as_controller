#ifndef ASNAV_API_H
#define ASNAV_API_H

#include "mavros_msgs/CommandBool.h"
#include <mavros_msgs/CommandLong.h>
#include "mavros_msgs/PositionTarget.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/State.h"
#include "nav_msgs/Odometry.h"
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>
#include <yolov8_ros_msgs/BoundingBoxes.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <string>

#include <actionlib/client/simple_action_client.h> 


class ASNAV
{
    public:
    // ====== 导航轴策略（2026-10-05）======
    // 决定 Z / Yaw 是否交给规划器：
    //   启用的轴 → 用规划器输出，接口形参被忽略
    //   未启用的轴 → 用接口形参（Z 走 PX4 位置控制，Yaw 直接作为目标角）
    // 编号按 bit 语义：bit0 = Z，bit1 = Yaw
    enum NavMode {
        NAV_FULL     = 0,   // Z:规划器  Yaw:规划器   （科研复现：全交给规划器）
        NAV_Z_ONLY   = 1,   // Z:规划器  Yaw:形参
        NAV_YAW_ONLY = 2,   // Z:形参    Yaw:规划器   （只开放机头朝向）
        NAV_LEVEL    = 3    // Z:形参    Yaw:形参     （比赛：PX4 锁高 + 机头锁 0）
    };
    explicit ASNAV(ros::NodeHandle& nh_);
    ~ASNAV();
    bool takeoff(float height);
    bool position(float x, float y, float z, float yaw, float tol = 0.2f);
    bool positionSmooth(float target_x, float target_y, float target_z, float tol, float hover_sec = 0.0f);
    bool navigation(float x, float y, float z, float yaw, float tol = 0.2f, float hover_sec = 1.0f);
    bool navigationWithPosition(float x, float y, float z, float yaw, float tol, float hover_sec);
    // nav_mode: 见上方 NavMode；-1（默认）→ 用 launch 的 nav_default_mode
    bool navigationEgo(float x, float y, float z, float yaw, float tol = 0.2f, bool stop_at_goal = false, int nav_mode = -1);
    bool navigationSuper(float x, float y, float z, float yaw, float tol = 0.2f, bool stop_at_goal = false, int nav_mode = -1);
    // rviz 打点接口同样支持 nav_mode：未启用的轴取 fly_height / 0.0f 作为锁定值
    bool navigationEgoRviz(int nav_mode = -1);
    bool navigationSuperRviz(int nav_mode = -1);
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
    bool pwmControl(int pwm_channel_5, int pwm_channel_6, int pwm_channel_7 = 50);
    bool putShoot(float x, float y, float z, float yaw, float tol);
    bool putShootSimple(float x, float y, float z, float yaw, float tol);
    bool putShootPlus(float x, float y, float z, float yaw, float tol);
    bool arTrackLanding(float ground_z = 0.0f, float altitude = 1.0f, float max_error = 0.20f, float vel_set = 0.15f, float camera_offset_x = 0.0f, float camera_offset_y = 0.0f);

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
    void mavros_local_velocity_cb(const geometry_msgs::TwistStamped::ConstPtr& msg);
    //ego回调
    void ego_planner_pos_cmd_cb(const quadrotor_msgs::PositionCommand::ConstPtr& msg);
    //yolo回调
    void yolo_info_cb(const yolov8_ros_msgs::BoundingBoxes::ConstPtr& msg);
    void yolo_d435i_info_cb(const yolov8_ros_msgs::BoundingBoxes::ConstPtr& msg);  // D435i前视YOLO回调
    //ar标签回调
    void ar_pose_cb(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);
    ros::NodeHandle nh_;
    //mavros相关组件
    ros::Subscriber mavros_state_sub_, mavros_local_position_pose_sub_, mavros_local_velocity_sub_;
    ros::Publisher mavros_setpoint_raw_local_pub_;
    ros::ServiceClient set_mode_client_;
    ros::ServiceClient mavros_cmd_command_client_;
    //ego相关组件
    ros::Subscriber ego_planner_pos_cmd_sub_;  
    ros::Publisher goal_pub_;
    //yolo相关组件
    ros::Subscriber yolo_info_sub_;
    ros::Subscriber yolo_d435i_info_sub_;  // D435i前视相机YOLO订阅
    //ar标签相关组件
    ros::Subscriber ar_pose_sub_;
    //mavros变量
    bool is_offboard, is_yaw_finished = false;
    geometry_msgs::Point current_position;
    geometry_msgs::Vector3 current_velocity;
    mavros_msgs::PositionTarget target_position;
    mavros_msgs::State current_state;
    float current_yaw;
    ros::Time yaw_finish_time;

    // mavros_msgs::CommandLong lib_ctrl_pwm;

    // EGO 规划器指令缓存
    quadrotor_msgs::PositionCommand ego_cmd_;
    bool ego_cmd_received_ = false;
    bool goal_sent_;
    bool ego_rviz_mode_ = false;  // navigationEgo 的 rviz 测试模式：不发 goal、不判到达

    // SUPER 规划器指令缓存（navigationSuper / navigationSuperRviz 使用）
    quadrotor_msgs::PositionCommand super_cmd_;
    bool super_cmd_received_ = false;
    ros::Time last_super_msg_time_;
    bool super_goal_sent_;
    bool super_rviz_mode_;        // navigationSuper 的 rviz 测试模式：不发 goal、不判到达
    ros::Publisher super_goal_pub_;
    ros::Subscriber super_planner_pos_cmd_sub_;
    void super_planner_pos_cmd_cb(const quadrotor_msgs::PositionCommand::ConstPtr& msg);

    // navigationSuper 参数（从launch加载）
    float super_kp_outer_;       // Kp: 位置误差 → 速度修正
    float super_kv_outer_;       // Kv: 速度误差阻尼 (v_ref→v_actual)
    float super_ki_outer_;       // Ki: 位置误差积分
    float super_ff_gain_;        // 速度前馈系数 XY (2026-10-03 参数化; fuel_nav 为 0.5)
    float super_ff_gain_z_;      // 速度前馈系数 Z  (2026-10-03; fuel_nav 的 Z 无前馈, 默认 0)
    float super_max_vel_;        // 速度指令限幅 (m/s) XY
    float super_max_vel_z_;      // Z 速度指令限幅 (m/s)
    float super_max_integral_;   // 积分抗饱和上限
    float super_traj_timeout_;   // 轨迹超时时间 (s)

    // navigationSuper 运行时状态
    double integral_spx_;         // X 位置误差积分
    double integral_spy_;         // Y 位置误差积分
    double integral_spz_;         // Z 位置误差积分
    double super_hold_px_ = 0.0;      // 位置保持锁存点(参考跳动时锁定, 不跟跳动参考)
    double super_hold_py_ = 0.0;
    bool super_hold_active_ = false;  // holdfix2: HOLD 位置锁存已初始化(进入HOLD时锁存一次)
    ros::Time super_tol_entry_time_;  // 进入容差时刻
    bool super_tol_timing_;           // 是否正在 debounce 计时
    // yawfix（2026-10-05 恢复）: 位置到位后等 yaw 也转入容差，仅 use_yaw 时启用
    bool super_yaw_finishing_ = false; // 正在 yaw 收尾阶段
    bool super_yaw_timing_ = false;    // yaw 到位防抖计时中
    ros::Time super_yaw_entry_time_;   // yaw 到位防抖计时起点

    //其他变量
    double start_planning_time, finish_planning_time;

    yoloBox yolo_box_info;            // 单目(下视)YOLO结果
    yoloBox yolo_d435i_box_info_;     // D435i(前视)YOLO结果
    std::string target_class_name;
    float integral_error_x, integral_error_y, last_err_x, last_err_y;
    ros::Time last_yolo_time_;
    ros::Time last_yolo_d435i_time_;  // D435i最后识别时间
    //ar标签跟踪变量
    bool ar_marker_found_;
    float ar_position_detec_x_, ar_position_detec_y_, ar_position_detec_z_;

    // navigationEgo 参数（从launch加载）
    float ego_kp_outer_;       // Kp: 位置误差 → 速度修正
    float ego_kv_outer_;       // Kv: 速度误差阻尼 (v_ref→v_actual)
    float ego_ki_outer_;       // Ki: 位置误差积分
    float ego_ff_gain_;        // 速度前馈系数 XY (2026-10-04 对齐 navigationSuper; fuel_nav 为 0.5)
    float ego_ff_gain_z_;      // 速度前馈系数 Z  (2026-10-04 对齐 navigationSuper, 默认 0)
    float ego_max_vel_;        // 速度指令限幅 XY (m/s)
    float ego_max_vel_z_;      // 速度指令限幅 Z  (m/s)
    float ego_max_integral_;   // 积分抗饱和上限
    float ego_traj_timeout_;   // 轨迹超时时间 (s)

    // positionSmooth 步长控制参数（从launch加载）
    double smooth_step_xy_;        // 正常步长 (m/cycle)，默认 0.18
    double smooth_slow_step_xy_;   // 接近目标时的减速步长 (m/cycle)，默认 0.01
    double smooth_slow_dist_;      // 触发减速的距离阈值 (m)，默认 0.5

    // navigationEgo 运行时状态（2026-10-05 框架对齐 navigationSuper）
    int nav_default_mode_ = 0;     // nav_mode < 0 时使用的全局默认（launch: nav_default_mode）
    double integral_egox_;         // X 位置误差积分
    double integral_egoy_;         // Y 位置误差积分
    double integral_egoz_;         // Z 位置误差积分
    ros::Time last_ego_msg_time_;  // 最近一次收到 ego 消息的时间戳
    double ego_hold_px_ = 0.0;     // 位置保持锁存点（HOLD 时锁定一次，不跟跳动参考）
    double ego_hold_py_ = 0.0;
    bool ego_hold_active_ = false; // HOLD 位置锁存已初始化
    ros::Time ego_tol_entry_time_; // 进入容差时刻
    bool ego_tol_timing_;          // 是否正在 debounce 计时
};
#endif