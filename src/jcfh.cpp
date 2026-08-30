/***************************************************************************************************************************
 *
 * Author: Amy
 * Time: 2023.01.16
 * Description: 根据预定航点进行避障飞行，还可设置达到航点后是否控制舵机投放，或者识别打靶。
 ***************************************************************************************************************************/
#include "jcfh.h"

PX4RosNav::PX4RosNav(const ros::NodeHandle &nh_private, int argc, char **argv) : nh_private_(nh_private)
{
  int flight_type, point_num;

  nh_private_.param<float>("fly_height", fly_height, 1.0);
  nh_private_.param<string>("mission_type", mission_type, "NULL");

  nh_private_.param<float>("hover_low_height", hover_low_height, 0.2);
  nh_private_.param<float>("mission_interval", mission_interval, 10.0);

  nh_private_.param<int>("flight_type", flight_type, 0);

  nh_private_.param<bool>("is_insert_waypoint", is_insert_waypoint, true);
  nh_private_.param<bool>("is_auto_land", is_auto_land, false);

  if (mission_type == "blink")
  {
    nh_private_.param<int>("detect_waypoint_id", detect_waypoint_id, 1);

    nh_private_.param<float>("detect_timeout", detect_timeout, 20.0);
    nh_private_.param<float>("target_area_offset_z", target_area_offset_z, -0.25);
    LogVerbose("start to init Detect");
    detect_letter = DetectABC(argc, argv);

    ROS_WARN("Init Detect");

    detect_result = "";
    target_letter = "";
    is_detect = false;

    detect_loop_timer = nh_private_.createTimer(ros::Duration(0.1), &PX4RosNav::DetectTimer, this, true);
    is_init_detect = true;
  }

  if (flight_type == 1)
  {
    nh_private_.param<float>("pub_delay", pub_delay, 2.0);
    nh_private_.param<int>("point_num", point_num, 0);

    for (int i = 0; i < point_num; i++)
    {
      Point point;
      nh_private_.param<float>("point" + to_string(i) + "_x", point.x, 0.0);
      nh_private_.param<float>("point" + to_string(i) + "_y", point.y, 0.0);
      nh_private_.param<float>("point" + to_string(i) + "_z", point.z, 0.0);
      waypoints.push_back(point);
    }
  }

  state_sub = nh_private_.subscribe("/mavros/state", 1, &PX4RosNav::StateCallback, this, ros::TransportHints().tcpNoDelay());
  pose_sub_ = nh_private_.subscribe("/mavros/local_position/pose", 1, &PX4RosNav::PoseCallback, this, ros::TransportHints().tcpNoDelay());
  cmd_vel_sub_ = nh_private_.subscribe("/cmd_vel", 1, &PX4RosNav::PlannerVelCallback, this, ros::TransportHints().tcpNoDelay());
  status_sub_ = nh_private_.subscribe("/move_base/status", 1, &PX4RosNav::PlannerStatusCallback, this, ros::TransportHints().tcpNoDelay());
  waypoint_pub = nh_private_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
  gpio_cmd_pub = nh_private_.advertise<std_msgs::String>("/drop_cmd", 10);                      // 发布投放物块 话题命令
  cmdloop_timer_ = nh_private_.createTimer(ros::Duration(0.1), &PX4RosNav::FlyCmdLooper, this); // 定义运行周期为0.1s

  is_offboard = false;
  is_planning_cannel = false;

  fly_mode = UP;

  waypoint_id = -1;

  current_position = geometry_msgs::Point();
  target_position = geometry_msgs::Point();
  planner_velxy_posz = geometry_msgs::Point();
  // planner_pos = geometry_msgs::Point();
  // planner_vel = geometry_msgs::Point();

  target_position.z = fly_height;
  planner_velxy_posz.z = fly_height;
  // planner_pos.z = fly_height;
  flightControl = FlightControl("");
}

PX4RosNav::~PX4RosNav()
{
  // Destructor
  if (mission_type == "blink")
  {
    detect_letter.destroy();
  }
}

void PX4RosNav::StateCallback(const mavros_msgs::State &msg)
{
  if (!is_offboard)
  {
    if (msg.armed && msg.mode == "OFFBOARD")
    {
      is_offboard = true;
      ROS_WARN("set mode: OFFBOARD");
      ROS_WARN("Fly to the desire height.");
      start_planning_time == ros::Time::now().toSec();
    }
    else
      is_offboard = false;
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
    break;
  case HOVER_LOW:
    if (mission_type == "drop")
    {
      if ((ros::Time::now().toSec() - start_mission_time > mission_interval))
      {
        ROS_WARN(" Drop mission finished, Back to the desire height.");

        is_mission_finished = true;
        target_position.z = fly_height;
        fly_mode = UP;
      }
    }
    else if (mission_type == "blink")
    {
      if (waypoint_id == detect_waypoint_id)
      {
        if (detect_result != "") // detected
        {
          ROS_WARN("Detected:%s.", detect_result.c_str());
          detect_result = "";
          is_mission_finished = true;
          target_position.z = fly_height;
          fly_mode = UP;

          // turn front the camera
          ROS_WARN("camera: turn front.");
          send_gpio_cmd("Front");
        }
      }
      else
      {
        if (start_blink_time != -1 && ros ::Time::now().toSec() - start_blink_time > mission_interval)
        {
          start_blink_time = -1;
          if (waypoints.size() > 0)
          {
            target_position.z = fly_height;
            fly_mode = UP;
          }
          else if (waypoints.size() == 0)
          {
            if (is_auto_land)
            {
              auto_land();
              fly_mode = DISAMRED;
            }
          }
        }
      }
    }

    flightControl.setpoint_raw_local_pos(target_position);

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
      ROS_WARN("Reached: height: %.2f at %.2f, fly to the waypoint.", fly_height, finish_planning_time - start_planning_time);

      if (waypoints.size() > 0)
      {
        publish_waypoint();
      }
      else
      {
        ROS_WARN("Reached: height: %.2f at %.2f, wait for the new waypoint.", fly_height, finish_planning_time - start_planning_time);
      }
    }
    break;

  case DOWN:
    if (abs(current_position.z - target_position.z) < 0.05)
    {
      target_position.x = current_position.x;
      target_position.y = current_position.y;
      target_position.z = current_position.z;
      fly_mode = HOVER_LOW;
      ROS_WARN("Reached: height: %.2f, ready to execute the mission: %s", target_position.z, mission_type.c_str());

      start_mission_time = ros::Time::now().toSec();
      if (mission_type == "drop")
      {
        send_gpio_cmd("PUT" + std::to_string(waypoint_id));
      }
      if (mission_type == "blink")
      {
        if (waypoint_id == detect_waypoint_id)
        { // turn down the camera
          ROS_WARN("camera: turn down.");
          send_gpio_cmd("Down");

          if (is_detect == false)
            is_detect = true;
        }
        else
        {
          start_blink_time = ros::Time::now().toSec();
          ROS_WARN("blink: publish.");
          send_gpio_cmd("Blink");
        }
      }
    }
    break;
  case PLANNING:
    if (abs(waypoints.front().x - current_position.x) < 0.2 && abs(waypoints.front().y - current_position.y) < 0.2 && abs(waypoints.front().z - current_position.z) < 0.2)
    {
      if (!is_near_target)
      {
        ROS_WARN("Near the target.");
        is_near_target = true;
      }
    }
    break;
  }
}

/*
  actionlib_msgs::GoalStatusArray:

    Header header
      uint32 seq
      time stamp
      string frame_id
    GoalStatus[] status_list
      actionlib_msgs/GoalID goal_id
        time stamp
        string id
      uint8 status
      string text

      uint8 status value:
        uint8 PENDING         = 0   # The goal has yet to be processed by the action server
        uint8 ACTIVE          = 1   # The goal is currently being processed by the action server
        uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing
                                    #   and has since completed its execution (Terminal State)
        uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)
        uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due
                                    #    to some failure (Terminal State)
        uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,
                                    #    because the goal was unattainable or invalid (Terminal State)
        uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing
                                    #    and has not yet completed execution
        uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,
                                    #    but the action server has not yet confirmed that the goal is canceled
        uint8 RECALLED        = 8   # The goal received a cancel request before it started executing
                                    #    and was successfully cancelled (Terminal State)
        uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be
                                    #    sent over the wire by an action server
 */
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
    if (msg.status_list[i].status == 1) // find the active goal
    {
      int end_index = msg.status_list[i].goal_id.id.find("-", 12);
      id_str = msg.status_list[i].goal_id.id.substr(11, end_index - 11);
      active_id = std::stoi(id_str);
      if (last_active_id != active_id) // start a new waypoint
      {
        last_active_id = active_id;
        ROS_WARN("Start the new waypoint: %d", waypoint_id);

        is_mission_finished = false;

        if (waypoints.size() == 1 && (!is_insert_waypoint || !is_planning_cannel))
          is_mission_finished = true;
      }
      break;
    }
    else if (msg.status_list[i].status == 2) // Canceled or Changed the waypoint.
    {
      if (active_id != -1)
      {
        int end_index = msg.status_list[i].goal_id.id.find("-", 12);
        id_str = msg.status_list[i].goal_id.id.substr(11, end_index - 11);

        if (active_id == std::stoi(id_str))
        {
          if (is_insert_waypoint)
          {
            is_planning_cannel = true;
            active_id = -1;
            finish_planning_time = ros::Time::now().toSec();
            ROS_WARN("Canceled the waypoint: %d", std::stoi(id_str));
          }
          else
            ROS_WARN("Changed the waypoint: %d", std::stoi(id_str));
          break;
        }
      }
    }
    else if (msg.status_list[i].status == 3 || msg.status_list[i].status == 4) // succeeded or aborted goal
    {
      if (active_id != -1)
      {
        int end_index = msg.status_list[i].goal_id.id.find("-", 12);
        id_str = msg.status_list[i].goal_id.id.substr(11, end_index - 11);

        if (active_id == std::stoi(id_str)) // lastest active goal is succeeded or aborted.
        {
          active_id = -1;
          target_position.x = current_position.x;
          target_position.y = current_position.y;
          fly_mode = HOVER_HIGH;

          if (msg.status_list[i].status == 3)
          {
            finish_planning_time = ros::Time::now().toSec();
            ROS_WARN("Waypoint: %s.", id_str.data());
            ROS_WARN("Finish the waypoint: %d, Spend time: %f", waypoint_id, finish_planning_time - start_planning_time);
          }
          else if (msg.status_list[i].status == 4)
            ROS_WARN("Failed to finish the waypoint: %d, skip it.", std::stoi(id_str));

          start_planning_time = -1;

          if (is_offboard && waypoints.size() > 0)
          {
            if (!is_insert_waypoint || !is_planning_cannel)
            {
              waypoints.pop_front();
            }
            is_planning_cannel = false;

            if (waypoints.size() == 0)
            {
              if (mission_type != "blink")
              {
                ROS_WARN("Finish all Waypoints.");
                if (is_auto_land)
                {
                  fly_mode = DISAMRED;
                  auto_land();
                }
              }
              else if (target_letter == "")
              {
                ROS_WARN("Back started point, auto land.");
                if (is_auto_land)
                {
                  fly_mode = DISAMRED;
                  auto_land();
                }
              }
            }
          }
        }
      }
      else if (fly_mode == HOVER_HIGH) // finished the waypoint, waiting for new waypoint
      {
        if (mission_type == "NULL")
        {
          if (pub_delay != -1 && finish_planning_time != -1 && ros::Time::now().toSec() - finish_planning_time > pub_delay)
          {
            publish_waypoint();
            break;
          }
        }
        else if (mission_type == "drop")
        {
          if (!is_mission_finished)
          {
            fly_mode = DOWN;
            ROS_WARN("Begin to fly down.");
            target_position.z = hover_low_height;
          }
          else
          {
            if (pub_delay != -1 && finish_planning_time != -1 && ros::Time::now().toSec() - finish_planning_time > pub_delay)
            {
              publish_waypoint();
              break;
            }
          }
        }
        else if (mission_type == "blink")
        {
          if (waypoint_id < detect_waypoint_id || waypoint_id > detect_waypoint_id + 2)
          {
            if (pub_delay != -1 && finish_planning_time != -1 && ros::Time::now().toSec() - finish_planning_time > pub_delay)
            {
              ROS_WARN("Publish delay:%.2f, %.2f.", ros::Time::now().toSec() - finish_planning_time, pub_delay);
              publish_waypoint();
              break;
            }
          }
          else
          {
            if (detect_result == "") // detecting
            {
              if (waypoint_id == detect_waypoint_id)
              {
                target_position.z = hover_low_height;
                fly_mode = DOWN;
                ROS_WARN("Begin to fly down.");
              }
              else if (is_detect == false)
              {
                if (finish_planning_time != -1)
                  is_detect = true;
              }
              else if (finish_planning_time != -1 && ros::Time::now().toSec() - finish_planning_time > detect_timeout)
              {
                is_detect = false;
                ROS_WARN("Detecting timeout, fly to next detecting point.");
                if (start_planning_time == -1)
                {
                  publish_waypoint();
                  break;
                }
              }
            }
            else
            {
              if (waypoint_id == detect_waypoint_id)
              {
                detect_result = "";
              }
              else if (waypoint_id > detect_waypoint_id)
              {
                static float target_z = 0;
                if (target_letter != "")
                {
                  Point point;
                  int index = detect_result.find(",");
                  if (index != -1)
                  {
                    string delta_y_str = detect_result.substr(0, index - 1);
                    string delta_z_str = detect_result.substr(index + 1, detect_result.length());
                    ROS_WARN("target relative position(y,z):%s, %s", delta_y_str.c_str(), delta_z_str.c_str());
                    point.x = target_position.x;
                    point.y = target_position.y - stof(delta_y_str);
                    point.z = target_position.z;
                    // target_z = target_position.z + stof(delta_z_str) + target_area_offset_z;
                    target_z = target_position.z + target_area_offset_z;
                    ROS_WARN("target relative position(x,y,z,z):%.2f, %.2f, %.2f, %.2f", point.x, point.y, point.z, target_z);
                    ROS_WARN("waypoints size: %d.", waypoints.size());
                    // waypoints.clear();
                    waypoints.push_front(point);
                    target_letter = "";
                    if (start_planning_time == -1)
                    {
                      publish_waypoint();
                      break;
                    }
                  }
                }
                else
                {
                  fly_mode = DOWN;
                  if (target_z > 0)
                    target_position.z = target_z;
                  ROS_WARN("Begin to fly down to %.2f.", target_position.z);
                }
              }
            }
          }
        }
      }
    }
  }
}

void PX4RosNav::DetectTimer(const ros::TimerEvent &event)
{
  while (is_init_detect)
  {
    if (is_detect)
    {
      if (waypoint_id == detect_waypoint_id)
      {
        ROS_INFO("Detecting Target...");
        detect_result = detect_letter.DetectOnce("", is_detect);
        target_letter = detect_result;
      }
      else if (target_letter != "")
      {
        // ROS_INFO("Detecting %s.\n", target_letter.c_str());
        detect_result = detect_letter.DetectOnce(target_letter, is_detect);
      }

      if (detect_result != "")
      {
        is_detect = false;
      }
    }
    else
    {
      detect_letter.DetectOnce("", is_detect);
    }
    ros::Duration(0.2).sleep();
  }
}

void PX4RosNav::publish_waypoint()
{
  finish_planning_time = -1;
  if (is_offboard && waypoints.size() > 0)
  {
    geometry_msgs::PoseStamped waypoint;

    waypoint.header.stamp = ros::Time::now();
    waypoint.header.frame_id = "map";

    waypoint.pose.position.x = waypoints.front().x;
    waypoint.pose.position.y = waypoints.front().y;
    waypoint.pose.position.z = waypoints.front().z;
    waypoint.pose.orientation.x = 0;
    waypoint.pose.orientation.y = 0;
    waypoint.pose.orientation.z = 0;
    waypoint.pose.orientation.w = 1;

    start_planning_time = ros::Time::now().toSec();
    // last_planning_time = -1;

    waypoint_pub.publish(waypoint);
    fly_mode = PLANNING;
    ROS_WARN("Publish Waypoint %d", ++waypoint_id);
  }
}

void PX4RosNav::PlannerVelCallback(const geometry_msgs::Twist &msg)
{
  if (mission_type == "NULL" && fly_mode == HOVER_HIGH)
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
  // flightControl.setpoint_raw_local_pos_vel(planner_pos, planner_vel);
}

void PX4RosNav::send_gpio_cmd(string msg)
{
  std_msgs::String gpio_cmd;
  gpio_cmd.data = msg;
  gpio_cmd_pub.publish(gpio_cmd);
}

void PX4RosNav::auto_land()
{
  flightControl.set_mode("POSCTL");
  ros::Duration(0.5).sleep();
  flightControl.set_mode("AUTO.LAND");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "jcfh");
  LogVerbose("ros init");
  ros::NodeHandle nh_private("~");

  PX4RosNav PX4RosNav(nh_private, argc, argv);

  ros::MultiThreadedSpinner s(6);
  ros::spin(s);
  return 0;
}
