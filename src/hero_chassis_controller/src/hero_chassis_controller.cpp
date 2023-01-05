#include "hero_chassis_controller/hero_chassis_controller.h"
#include <pluginlib/class_list_macros.hpp>
#include <pid.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <trajectory_msgs/JointTrajectory.h>

namespace hero_chassis_controller {
bool HeroChassisController::init(hardware_interface::EffortJointInterface *effort_joint_interface,
                                   ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh, const control_toolbox::Pid &pid) {
  pid_controller_ = pid;                                 
  front_left_joint_ =
      effort_joint_interface->getHandle("left_front_wheel_joint");
  front_right_joint_ =
      effort_joint_interface->getHandle("right_front_wheel_joint");
  back_left_joint_ = effort_joint_interface->getHandle("left_back_wheel_joint");
  back_right_joint_ =
      effort_joint_interface->getHandle("right_back_wheel_joint");

if (!pid_controller_.init(ros::NodeHandle(n, "pid")))
    return false;

    sub_command_ = n.subscribe("/cmd_vel", 1, &HeroChassisController::receiveCmd, this);
  return true;
}

ros::Publisher pub;

void PlatformCtrlNode::receiveCmd(const geometry_msgs::Twist& twist)
{
  ros::Rate loopRate(1);
  
	double x = twist.linear.x ;
	double y = twist.linear.y ;
	double z = twist.linear.z ;
	double omiga = twist.angular.z ;
	
}

void JointVelocityController::setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min)
{
  pid_controller_.setGains(p,i,d,i_max,i_min);
}

void JointVelocityController::getGains(double &p, double &i, double &d, double &i_max, double &i_min)
{
  pid_controller_.getGains(p,i,d,i_max,i_min;
}

void HeroChassisController::update(const ros::Time &time, const ros::Duration &period) {
  /*double tau = 0.2;  // torque
  static double cmd_[6][4] = {{tau, tau, tau, tau}, //  forward
                              {-2 * tau, -2 * tau, -2 * tau, -2 * tau}, //  backward
                              {-tau, tau, tau, -tau}, //  left
                              {2 * tau, -2 * tau, -2 * tau, 2 * tau}, //  right
                              {2 * tau, -2 * tau, 2 * tau, -2 * tau}, //  clockwise
                              {-tau, tau, -tau, tau}};  //  counterclockwise
  if ((time - last_change_).toSec() > 2) {
    state_ = (state_ + 1) % 6;
    last_change_ = time;
  }
*/
  front_left_joint_.setCommand(cmd_[state_][0]);
  front_right_joint_.setCommand(cmd_[state_][1]);
  back_left_joint_.setCommand(cmd_[state_][2]);
  back_right_joint_.setCommand(cmd_[state_][3]);
}

PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase)
}



