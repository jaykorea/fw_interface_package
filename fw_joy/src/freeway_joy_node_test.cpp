#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Path.h>
#include <math.h>
#include <sstream>
#include "geometry_msgs/Twist.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include "freeway_joyfw/stm_fw_msg.h"
#include "freeway_joyfw/stm_am_msg.h"
#include "freeway_joyfw/stm_fw_srv.h"

class Freeway_Joy_Fw
{
  public:
    freeway_joyfw::stm_fw_msg stm_msg;
    freeway_joyfw::stm_am_msg am_msg;
    geometry_msgs::Twist cmd_vel_msg;
    actionlib_msgs::GoalID empty_goal;
    Freeway_Joy_Fw(ros::NodeHandle *n)
    {
      cmd_pub = n->advertise<geometry_msgs::Twist>("cmd_vel", 10);
      am_mode_pub = n->advertise<freeway_joyfw::stm_am_msg>("freeway/am_status", 10);
      move_base_cancel_pub = n->advertise<actionlib_msgs::GoalID>("move_base/cancel", 10);
      diag_sub = n->subscribe("freeway/diagnostics", 1000, &Freeway_Joy_Fw::get_diagnostics_cb, this);
    }

    bool am_mode_cb(freeway_joyfw::stm_fw_srv::Request &req, freeway_joyfw::stm_fw_srv::Response &res);

    void get_diagnostics_cb(const freeway_joyfw::stm_fw_msg &diag_msg) {
      stm_msg = diag_msg;
      if (stm_msg.am_status == true && stm_msg.e_stop_status == true)
      {
        cmd_vel_msg.linear.x = diag_msg.cmd_vel_mcu.linear.x;
        cmd_vel_msg.angular.z = diag_msg.cmd_vel_mcu.angular.z;
        cmd_pub.publish(cmd_vel_msg);
      }
      else if (stm_msg.e_stop_status == false)
      {
        cmd_vel_msg.linear.x = diag_msg.cmd_vel_mcu.linear.x;
        cmd_vel_msg.angular.z = diag_msg.cmd_vel_mcu.angular.z;
        cmd_pub.publish(cmd_vel_msg);
        move_base_cancel_pub.publish(empty_goal);
      }
    }

  private:
    ros::Publisher cmd_pub;
    ros::Publisher move_base_cancel_pub;
    ros::Publisher am_mode_pub;
    ros::Subscriber diag_sub;
};

bool Freeway_Joy_Fw::am_mode_cb(freeway_joyfw::stm_fw_srv::Request &req, freeway_joyfw::stm_fw_srv::Response &res)
{
  res.result = req.am_mode;
  am_msg.am_status2=res.result;
  am_mode_pub.publish(am_msg);
  ROS_INFO("res.result : %d", res.result);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "freeway_joy_node");
  ros::NodeHandle n;

  Freeway_Joy_Fw Fj = Freeway_Joy_Fw(&n);
  ros::ServiceServer service = n.advertiseService("am_mode", &Freeway_Joy_Fw::am_mode_cb, &Fj);
  ros::Rate loop_rate(20);

  while (ros::ok())
  {
    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
