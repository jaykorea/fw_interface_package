#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/LaserScan.h>
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
    Freeway_Joy_Fw(ros::NodeHandle *n)
    {
      cmd_pub = n->advertise<geometry_msgs::Twist>("cmd_vel", 10);
      am_mode_pub = n->advertise<freeway_joyfw::stm_am_msg>("freeway/am_status", 10);
      move_base_flex_cancel_pub = n->advertise<actionlib_msgs::GoalID>("move_base_flex/move_base/cancel", 10);
      diag_sub = n->subscribe("freeway/diagnostics", 1000, &Freeway_Joy_Fw::get_diagnostics_cb, this);
      input_scan_sub = n->subscribe("/scan_rp_filtered", 50, &Freeway_Joy_Fw::update, this);
      signal = 0;
      signal2 = 0;
      reset = 0;
    }

    bool am_mode_cb(freeway_joyfw::stm_fw_srv::Request &req, freeway_joyfw::stm_fw_srv::Response &res);

    void update(const sensor_msgs::LaserScan& input_scan)
    {
      //reset = signal;
      if (!input_scan.ranges.empty()) {
          float res_per_deg = (int)input_scan.ranges.size() / (float)360.0;
          float las_mid_ran = res_per_deg * (float)180.0;
          float deg_15 = floor(res_per_deg * (float)15.0);
          float* ran_arr = new float[4*int(floor(deg_15))]();

          for (unsigned int i = int(floor(las_mid_ran))-2*int(floor(deg_15)); i < int(floor(las_mid_ran))+2*int(floor(deg_15)); i++) {
            if(input_scan.ranges[i] < 0.5) {
              signal++;
            }
            if (signal >= 30) {
              signal = 30;
            }
            ran_arr[i-(int(floor(las_mid_ran))-2*int(floor(deg_15)))]=input_scan.ranges[i];
          }
          
          for (unsigned int i = 0; i < 4*int(floor(deg_15))-1; i++) {
            if (ran_arr[i] >= 0.5) {
              signal2++;
            }
          }
          if (signal2 >= 4*int(floor(deg_15))-2) {
            signal2 =0;
            signal = 0;
          }
          else if (signal2 < 4*int(floor(deg_15))-1) signal2 = 0;
          
          delete [] ran_arr;
         }
       }
    

    void get_diagnostics_cb(const freeway_joyfw::stm_fw_msg &diag_msg) {
      freeway_joyfw::stm_fw_msg stm_msg;
      geometry_msgs::Twist cmd_vel_msg;
      actionlib_msgs::GoalID empty_goal;
      stm_msg = diag_msg;
      if (stm_msg.am_status == true && stm_msg.e_stop_status == true)
      {
        if(signal >= 30) {
          if (diag_msg.cmd_vel_mcu.linear.x > 0.0) {
              cmd_vel_msg.linear.x = 0.0;
          }
          else {
            cmd_vel_msg.linear.x =  diag_msg.cmd_vel_mcu.linear.x;
          }

          cmd_vel_msg.angular.z = diag_msg.cmd_vel_mcu.angular.z;
          cmd_pub.publish(cmd_vel_msg);
          
        }
        else {
          cmd_vel_msg.linear.x = diag_msg.cmd_vel_mcu.linear.x;
          cmd_vel_msg.angular.z = diag_msg.cmd_vel_mcu.angular.z;
          cmd_pub.publish(cmd_vel_msg);
        }
      }
      else if (stm_msg.e_stop_status == false)
      {
        cmd_vel_msg.linear.x = diag_msg.cmd_vel_mcu.linear.x;
        cmd_vel_msg.angular.z = diag_msg.cmd_vel_mcu.angular.z;
        cmd_pub.publish(cmd_vel_msg);
        move_base_flex_cancel_pub.publish(empty_goal);
      }
    }

  private:
    ros::Publisher cmd_pub;
    ros::Publisher move_base_flex_cancel_pub;
    ros::Publisher am_mode_pub;
    ros::Subscriber diag_sub;
    ros::Subscriber input_scan_sub;
    uint32_t signal = 0;
    uint32_t signal2 = 0;
    uint32_t reset = 0;
};

bool Freeway_Joy_Fw::am_mode_cb(freeway_joyfw::stm_fw_srv::Request &req, freeway_joyfw::stm_fw_srv::Response &res)
{
  freeway_joyfw::stm_am_msg am_msg;
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
