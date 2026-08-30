
#include <iostream>
#include <list>
#include <string>

#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>
#include <actionlib_msgs/GoalStatusArray.h>

#include "flight_control.h"

using namespace std;

class PX4RosNav
{
public:
  /**
   *默认构造函数
   */
  PX4RosNav(const ros::NodeHandle &nh_private, int argc, char **argv);
  /**
   * 析构函数
   */
  ~PX4RosNav();

  /**
   * @brief      check healthiness of the avoidance system to trigger failsafe in
   *             the FCU
   * @param[in]  since_last_cloud, time elapsed since the last waypoint was
   *             published to the FCU
   * @param[in]  since_start, time elapsed since staring the node
   * @param[out] planner_is_healthy, true if the planner is running without
   *errors
   * @param[out] hover, true if the vehicle is hovering
   **/

private:
  struct Point
  {
    float x = 0, y = 0, z = 0;
  };
  enum Fly_Mode
  {
    DISAMRED,
    UP,
    DOWN,
    HOVER_HIGH,
    HOVER_LOW,
    PLANNING,
    HOVER_HIGH_AT_HOVERPOINT  
  };

  void FlyCmdLooper(const ros::TimerEvent &event);
  void PlannerVelCallback(const geometry_msgs::Twist &msg);
  void PlannerStatusCallback(const actionlib_msgs::GoalStatusArray &msg);
  void StateCallback(const mavros_msgs::State &msg);
  void PoseCallback(const geometry_msgs::PoseStamped &msg);
  void publish_waypoint();
  void auto_land();

  ros::NodeHandle nh_private_;
  ros::Timer cmdloop_timer_;
  ros::Publisher waypoint_pub;
  ros::Subscriber cmd_vel_sub_, state_sub, status_sub_, pose_sub_;
  ros::ServiceClient ctrl_pwm_client;

  float fly_height, pub_delay = -1, hovertimes;//mission_interval, target_area_offset_z , shot_height, hover_low_height;
  std::list<Point> waypoints;
  double start_planning_time = -1, finish_planning_time = -1, start_hover_time = -1, goal_tolerance;

  bool is_offboard, is_auto_land;
  bool is_near_target = false;

  int waypoint_id = -1, putpoint1, putpoint2, putpoint3;
  // int hover_waypoint_;
  // int descend_waypoint_;


  geometry_msgs::Point planner_velxy_posz;
  // geometry_msgs::Point planner_pos;
  // geometry_msgs::Point planner_vel;
  // double last_planning_time;
  geometry_msgs::Point current_position,
      target_position;

  Fly_Mode fly_mode = DISAMRED;

  FlightControl flightControl;
};
