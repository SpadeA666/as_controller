#include <iostream>

#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>

using namespace std;
using namespace mavros_msgs;

class FlightControl
{
public:
    FlightControl();
    FlightControl(string mavros_ns);
    ~FlightControl();

    void set_arm(bool is_arm);
    void set_mode(string mode);

    void setpoint_position_local(const geometry_msgs::Point pos);

    void setpoint_raw_local_pos(const geometry_msgs::Point pos, float yaw = 0, int frame = PositionTarget::FRAME_LOCAL_NED);
    void setpoint_raw_local_vel(const geometry_msgs::Point vel, float yaw_rate = 0, int frame = PositionTarget::FRAME_LOCAL_NED);
    void setpoint_raw_local_pos_vel(const geometry_msgs::Point pos, const geometry_msgs::Point vel, float yaw = 0, int frame = PositionTarget::FRAME_LOCAL_NED);

    void setpoint_raw_local_velxy_posz(const geometry_msgs::Point vel_xy_pos_z, float yaw = 0, int frame = PositionTarget::FRAME_LOCAL_NED);

    void setpoint_raw_local_vx_yawrate_posz(const geometry_msgs::Point vx_yawrate_posz, int frame = PositionTarget::FRAME_LOCAL_NED);

private:
    ros::NodeHandle nh_;
    ros::Publisher setpoint_raw_local_pub_;
    ros::Publisher setpoint_position_local_pub_;

    ros::Subscriber status_sub_, pose_sub_;

    string mavros_ns_;
};