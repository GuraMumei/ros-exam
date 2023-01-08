#ifndef HERO_CHASSIS_CONTROLLER_H
#define HERO_CHASSIS_CONTROLLER_H

#include <controller_interface/controller.h>
#include <control_msgs/JointControllerState.h>
#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <bits/stdc++.h>
#include <nav_msgs/Odometry.h>
#include<ros/time.h>
#include <ros/duration.h>
#include <controller_interface/multi_interface_controller.h>
#include <tf/tfMessage.h>
#include <geometry_msgs/Twist.h>
#include "tf2/LinearMath/Quaternion.h"
#include "tf/transform_datatypes.h"


namespace hero_chassis_controller
{

class HeroChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
  HeroChassisController() = default;
  ~HeroChassisController() override = default;

  bool init(hardware_interface::EffortJointInterface* effort_joint_interface, ros::NodeHandle& root_nh,
            ros::NodeHandle& controller_nh) override;
  void getGains(double& p, double& i, double& d, double& i_max, double& i_min);
  void setGains(const double& p, const double& i, const double& d, const double& i_max, const double& i_min);
  void odomCallback(const nav_msgs::Odometry &odom) ;
  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void receiveCmd(const geometry_msgs::Twist& twist);
  void updateOdometry(const ros::Time& time);

  ros::NodeHandle node_;
  
  
  
  //tf::TransformListener listener;
  ros::Publisher pub;
  ros::Subscriber sub_command_;
  ros::Subscriber odomcmd;
  hardware_interface::JointHandle front_left_joint_, front_right_joint_, back_left_joint_, back_right_joint_;
  std::vector<control_toolbox::Pid> pid_controller_;

  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry> > odom_pub_;
  std::shared_ptr< realtime_tools::RealtimePublisher<geometry_msgs::TransformStamped> > tf_odom_pub_;
  //static tf::TransformBroadcaster odom_broadcaster;
  double output[4] = { 0, 0, 0, 0 };
  int state_{};
  ros::Time last_change_;
  ros::Time current_time, last_time;

  double v_tx ;
  double v_ty ;
  double omiga ;
  double WHEEL_DIAMETER =0.07625;  // 轮子直径,单位：米
  double D_X =0.4;                 // 底盘Y轴上两轮中心的间距
  double D_Y =0.4;                 // 底盘X轴上两轮中心的间距
  double v_x=0;
  double v_y=0;
  double w=0;
  double r_wheel = WHEEL_DIAMETER / 2;
  double r_x = D_X / 2;
  double r_y = D_Y / 2;
  bool odomcontrol =false;
  double x =0.0 ;
  double y = 0.0;
  double th = 0.0;
  double d;
};
}// namespace hero_chassis_controller
#endif



