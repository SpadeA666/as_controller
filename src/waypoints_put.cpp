#include "waypoints_put.h"

PX4RosNav::PX4RosNav(const ros::NodeHandle &nh_private, int argc, char **argv) : nh_private_(nh_private)
{
  int flight_type, point_num;

  nh_private_.param<float>("fly_height", fly_height, 1.0);
  nh_private_.param<float>("hovertimes", hovertimes, 2.0);
  nh_private_.param<int>("flight_type", flight_type, 0);
  nh_private_.param<bool>("is_auto_land", is_auto_land, false);
  nh_private_.param<double>("goal_tolerance", goal_tolerance, 0.2);
  nh_private_.param<int>("putpoint1", putpoint1, 0);
  nh_private_.param<int>("putpoint2", putpoint2, 2);
  nh_private_.param<int>("putpoint3", putpoint3, 4);

  if (flight_type == 1)
  {
    nh_private_.param<float>("pub_delay", pub_delay, 2.0);
    nh_private_.param<int>("point_num", point_num, 0);

    for (int i = 0; i < point_num; i++)
    {
      Point point;
      nh_private_.param<float>("point" + to_string(i) + "_x", point.x, 0.0);
      nh_private_.param<float>("point" + to_string(i) + "_y", point.y, 0.0);
      nh_private_.param<float>("point" + to_string(i) + "_z", point.z, fly_height); // 默认使用飞行高度
      waypoints.push_back(point);
    }
  }

  state_sub = nh_private_.subscribe("/mavros/state", 1, &PX4RosNav::StateCallback, this, ros::TransportHints().tcpNoDelay());
  pose_sub_ = nh_private_.subscribe("/mavros/local_position/pose", 1, &PX4RosNav::PoseCallback, this, ros::TransportHints().tcpNoDelay());
  cmd_vel_sub_ = nh_private_.subscribe("/cmd_vel", 1, &PX4RosNav::PlannerVelCallback, this, ros::TransportHints().tcpNoDelay());
  status_sub_ = nh_private_.subscribe("/move_base/status", 1, &PX4RosNav::PlannerStatusCallback, this, ros::TransportHints().tcpNoDelay());
  waypoint_pub = nh_private_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
  cmdloop_timer_ = nh_private_.createTimer(ros::Duration(0.1), &PX4RosNav::FlyCmdLooper, this);

  is_offboard = false;
  is_near_target = false;

  fly_mode = UP;

  waypoint_id = -1;

  current_position = geometry_msgs::Point();
  target_position = geometry_msgs::Point();
  planner_velxy_posz = geometry_msgs::Point();

  target_position.z = fly_height;
  planner_velxy_posz.z = fly_height;
  flightControl = FlightControl("");
  
  start_planning_time = 0;
  finish_planning_time = 0;
  start_hover_time = 0;
}

PX4RosNav::~PX4RosNav()
{
}

void PX4RosNav::StateCallback(const mavros_msgs::State &msg)
{
  if (!is_offboard)
  {
    if (msg.armed && msg.mode == "OFFBOARD")
    {
      is_offboard = true;
      ROS_WARN("Set mode: OFFBOARD");
      ROS_WARN("Flying to desired height: %.2f m", fly_height);
      start_planning_time = ros::Time::now().toSec();
    }
    else
    {
      is_offboard = false;
    }
  }
}

void PX4RosNav::FlyCmdLooper(const ros::TimerEvent &event)
{
    static double add_pos_z = 0;
    static geometry_msgs::Point position_temp;
  
    switch (fly_mode)
    {
        case UP:
        case DOWN:
            position_temp = target_position;
            add_pos_z = (target_position.z - current_position.z) * 0.3;
            if (abs(add_pos_z) > 0.2)
            {
                position_temp.z = current_position.z + add_pos_z / abs(add_pos_z) * 0.3;
            }
            else if (abs(add_pos_z) > 0.05)
            {
                position_temp.z = current_position.z + add_pos_z;
            }
            flightControl.setpoint_raw_local_pos(position_temp);
            break;

        case HOVER_HIGH:
            flightControl.setpoint_raw_local_pos(target_position);
            if (waypoints.size() > 0)
            {
                publish_waypoint();
            }
            break;

        case HOVER_LOW:
            flightControl.setpoint_raw_local_pos(target_position);
            break;
            
        case HOVER_HIGH_AT_HOVERPOINT:
            flightControl.setpoint_raw_local_pos(target_position);
            // 检查是否悬停满hovertimes秒
            if (ros::Time::now().toSec() - start_hover_time > hovertimes)
            {
                ROS_WARN("%.2f hover at waypoint %d finished, continuing to next waypoint", hovertimes, waypoint_id);
                // 准备下一个航点
                if (waypoints.size() > 0)
                {
                    waypoints.pop_front();
                    publish_waypoint();
                }
                if (waypoints.size() == 0)
                {
                  if (is_auto_land)
                  {
                    ROS_WARN("All waypoints completed. Initiating auto land.");
                    fly_mode = DISAMRED;
                    auto_land();
                  }
                  else
                  {
                    ROS_WARN("All waypoints completed, waiting for new waypoints.");
                  }
                }
            }
            break;

        default:
            break;
    }
}

void PX4RosNav::PoseCallback(const geometry_msgs::PoseStamped &msg)
{
  current_position.x = msg.pose.position.x;
  current_position.y = msg.pose.position.y;
  current_position.z = msg.pose.position.z;

  switch (fly_mode)
  {
    case UP:
      if (abs(current_position.z - target_position.z) < 0.05)
      {
        target_position.x = current_position.x;
        target_position.y = current_position.y;
        target_position.z = current_position.z;
        fly_mode = HOVER_HIGH;
        finish_planning_time = ros::Time::now().toSec();
        ROS_WARN("Reached target height: %.2f m", fly_height);    
      }
      break;

    case DOWN:
      if (abs(current_position.z - target_position.z) < 0.05)
      {
        target_position.x = current_position.x;
        target_position.y = current_position.y;
        target_position.z = current_position.z;
      }
      break;

    case PLANNING:
      // 检测是否到达悬停点且尚未悬停
      // if (waypoint_id == hover_waypoint_ && !is_hovered) 
      // {
      //   if (abs(waypoints.front().x - current_position.x) < goal_tolerance && 
      //       abs(waypoints.front().y - current_position.y) < goal_tolerance && 
      //       abs(waypoints.front().z - current_position.z) < goal_tolerance)
      //   {
      //     ROS_WARN("Reached hover waypoint %d, hovering for 4 seconds", waypoint_id);
      //     fly_mode = HOVER_HIGH_AT_HOVERPOINT;
      //     start_hover_time = ros::Time::now().toSec();
      //   }
      // }
      // 检测是否到达投放点且尚未下降
      // else if (waypoint_id == descend_waypoint_ && !is_dropped) 
      // {
      //   if (abs(waypoints.front().x - current_position.x) < goal_tolerance && 
      //       abs(waypoints.front().y - current_position.y) < goal_tolerance && 
      //       abs(waypoints.front().z - current_position.z) < goal_tolerance)
      //   {
      //     ROS_WARN("Reached descend waypoint %d, descending to low height: %.2f m", waypoint_id, hover_low_height);
      //     fly_mode = DOWN;
      //     target_position.z = hover_low_height;
      //     is_dropped = true;
      //   }
      // }
      // 检测是否接近目标
      if (abs(waypoints.front().x - current_position.x) < goal_tolerance && 
          abs(waypoints.front().y - current_position.y) < goal_tolerance && 
          abs(waypoints.front().z - current_position.z) < goal_tolerance)
      {
        if (!is_near_target)
        {
          ROS_WARN("Near the target waypoint");
          is_near_target = true;
        }
      }
      break;
      
    case HOVER_HIGH_AT_HOVERPOINT:
      // 保持位置，由定时器处理时间
      break;
  }
}

void PX4RosNav::PlannerStatusCallback(const actionlib_msgs::GoalStatusArray &msg)
{
    static int active_id = -1, last_active_id = -1;
    static string id_str;

    if (waypoint_id == -1 || !is_offboard)
    {
        return;
    }

    for (int i = 0; i < msg.status_list.size(); i++)
    {
        if (msg.status_list[i].status == 1)  // 目标正在执行
        {
            int end_index = msg.status_list[i].goal_id.id.find("-", 12);
            id_str = msg.status_list[i].goal_id.id.substr(11, end_index - 11);
            active_id = std::stoi(id_str);
          
            if (last_active_id != active_id)  // 启动新的航点
            {
                last_active_id = active_id;
                ROS_WARN("Starting new waypoint: %d", waypoint_id);          
            }
            break;
        }
        else if (msg.status_list[i].status == 3 || msg.status_list[i].status == 4)  // 完成或失败
        {
            if (active_id != -1)
            {
                // 任务完成或失败
                int end_index = msg.status_list[i].goal_id.id.find("-", 12);
                id_str = msg.status_list[i].goal_id.id.substr(11, end_index - 11);

                if (active_id == std::stoi(id_str))  // 当前激活的目标
                {
                    active_id = -1;
                    target_position.x = current_position.x;
                    target_position.y = current_position.y;

                    if (msg.status_list[i].status == 3)  // 成功
                    {
                        ROS_WARN("Finished waypoint: %d", waypoint_id);
                        finish_planning_time = ros::Time::now().toSec();
                        ROS_WARN("Time spent: %.2f seconds", finish_planning_time - start_planning_time);

                        if (waypoint_id == putpoint1 || putpoint2 || putpoint3 )
                        {
                          fly_mode = HOVER_HIGH_AT_HOVERPOINT;
                          start_hover_time = ros::Time::now().toSec();
                        }
                        
                        if (waypoints.size() > 0 && waypoint_id != putpoint1 && waypoint_id != putpoint2 && waypoint_id != putpoint3)
                        {
                          waypoints.pop_front();  // 弹出当前航点
                          fly_mode = HOVER_HIGH;
                        }

                        if (waypoints.size() == 0)
                        {
                          ros::Duration(1.0).sleep();
                         if (is_auto_land)
                         {
                           ROS_WARN("All waypoints completed. Initiating auto land.");
                           fly_mode = DISAMRED;
                           auto_land();
                         }
                         else
                         {
                           ROS_WARN("All waypoints completed, waiting for new waypoints.");
                         }
                        }
                        // 如果是悬停点设定点完成，准备悬停
                        // if (waypoint_id == hover_waypoint_ && !is_hovered)
                        // {
                        //     ROS_WARN("Waypoint %d (hover point) reached, starting 4-second hover", waypoint_id);
                        //     fly_mode = HOVER_HIGH_AT_HOVERPOINT;
                        //     start_hover_time = ros::Time::now().toSec();
                        // }
                        // 如果是投放点设定点完成，准备下降
                        // else if (waypoint_id == descend_waypoint_ && !is_dropped)
                        // {
                        //     ROS_WARN("Waypoint %d (descend point) reached, starting descent", waypoint_id);
                        //     fly_mode = DOWN;
                        //     target_position.z = hover_low_height;
                        //     is_dropped = true;
                        // }
                    }
                    else if (msg.status_list[i].status == 4)  // 失败
                    {
                        ROS_WARN("Failed to reach waypoint: %d", waypoint_id);
                    }

                    start_planning_time = -1;

                    // 继续处理下一个航点（悬停点和投放点除外）
                    //  if (waypoints.size() > 0) && !is_insert_waypoint && waypoint_id != hover_waypoint_ && waypoint_id != descend_waypoint_)
                    // {
                    //     waypoints.pop_front();  // 弹出当前航点
                    //     fly_mode = HOVER_HIGH;  // 飞向下一个航点
                    // }
                }
            }
        }
    }
}

void PX4RosNav::publish_waypoint()
{
  if (is_offboard && waypoints.size() > 0)
  {
    geometry_msgs::PoseStamped waypoint;

    waypoint.header.stamp = ros::Time::now();
    waypoint.header.frame_id = "map";

    waypoint.pose.position.x = waypoints.front().x;
    waypoint.pose.position.y = waypoints.front().y;
    waypoint.pose.position.z = waypoints.front().z;  // 使用航点的z坐标
    
    waypoint.pose.orientation.x = 0;
    waypoint.pose.orientation.y = 0;
    waypoint.pose.orientation.z = 0;
    waypoint.pose.orientation.w = 1;

    start_planning_time = ros::Time::now().toSec();
    finish_planning_time = -1;

    waypoint_pub.publish(waypoint);
    fly_mode = PLANNING;
    ROS_WARN("Publishing Waypoint %d at (%.2f, %.2f, %.2f)", 
             ++waypoint_id, 
             waypoints.front().x, 
             waypoints.front().y, 
             waypoints.front().z);
  }
}

void PX4RosNav::PlannerVelCallback(const geometry_msgs::Twist &msg)
{
  if (fly_mode == HOVER_HIGH)
    fly_mode = PLANNING;

  planner_velxy_posz.x = msg.linear.x;
  planner_velxy_posz.y = msg.linear.y;

  if (abs(planner_velxy_posz.x) <= 0.05 && abs(planner_velxy_posz.y) <= 0.05)
  {
    planner_velxy_posz.x = planner_velxy_posz.x * (0.12 - abs(planner_velxy_posz.x)) * 50;
    planner_velxy_posz.y = planner_velxy_posz.y * (0.12 - abs(planner_velxy_posz.y)) * 50;
  }

  if (fly_mode == PLANNING)
    flightControl.setpoint_raw_local_velxy_posz(planner_velxy_posz);
}

void PX4RosNav::auto_land()
{
  flightControl.set_mode("POSCTL");
  ros::Duration(0.5).sleep();
  flightControl.set_mode("AUTO.LAND");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "waypoints_put");
  ros::NodeHandle nh_private("~");

  PX4RosNav PX4RosNav(nh_private, argc, argv);

  ros::MultiThreadedSpinner s(6);
  ros::spin(s);
  return 0;
}