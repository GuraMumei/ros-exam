#include <pluginlib/class_list_macros.hpp>
#include <hero_chassis_controller/hero_chassis_controller.h>


namespace hero_chassis_controller
{

bool HeroChassisController::init(hardware_interface::EffortJointInterface* effort_joint_interface,
                                 ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  front_left_joint_ = effort_joint_interface->getHandle("left_front_wheel_joint");
  front_right_joint_ = effort_joint_interface->getHandle("right_front_wheel_joint");
  back_left_joint_ = effort_joint_interface->getHandle("left_back_wheel_joint");
  back_right_joint_ = effort_joint_interface->getHandle("right_back_wheel_joint");
//加载参数
  if (!controller_nh.getParam("WHEEL_DIAMETER", WHEEL_DIAMETER)) {
    ROS_ERROR("No joint given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("D_X", D_X)) {
    ROS_ERROR("No joint given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("D_Y", D_Y)) {
    ROS_ERROR("No joint given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("odomcontrol", odomcontrol )) {
    ROS_ERROR("No joint given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }


  // Load PID Controller using gains set on parameter server
  control_toolbox::Pid pid_controller;
    if (!pid_controller.init(ros::NodeHandle(node_, "controller/front_left/pid")))
      return false;
    pid_controller_.push_back(pid_controller);
    if (!pid_controller.init(ros::NodeHandle(node_, "controller/front_right/pid")))
      return false;
    pid_controller_.push_back(pid_controller);
    if (!pid_controller.init(ros::NodeHandle(node_, "controller/back_left/pid")))
      return false;
    pid_controller_.push_back(pid_controller);
    if (!pid_controller.init(ros::NodeHandle(node_, "controller/back_right/pid")))
      return false;
    pid_controller_.push_back(pid_controller);

  sub_command_ = node_.subscribe("/cmd_vel", 1, &HeroChassisController::receiveCmd, this);
  //odomcmd = node_.subscribe("/odom", 3, &HeroChassisController::odomCallback, this);
  // 发布里程计
  odom_pub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(controller_nh, "/odom", 100));
  //发布tf
  tf_odom_pub_.reset( new realtime_tools::RealtimePublisher<tf::tfMessage>(root_nh, "/tf", 100));
  tf_odom_pub_->msg_.transforms.resize(1);
    tf_odom_pub_->msg_.transforms[0].transform.translation.z = 0.0;
    tf_odom_pub_->msg_.transforms[0].child_frame_id = base_link_;
    tf_odom_pub_->msg_.transforms[0].header.frame_id = "odom";

  return true;
}

void HeroChassisController::receiveCmd(const geometry_msgs::Twist& twist)
{
  v_tx = twist.linear.x;
  v_ty = twist.linear.y;
  omiga = twist.angular.z;

}

/*void HeroChassisController::odomCallback(const nav_msgs::Odometry &odom)
{
  tf::Quaternion quat;
  tf::quaternionMsgToTF(odom.pose.pose.orientation, quat);
  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw); //进行转换
  d = yaw;
}
*/
void HeroChassisController::setGains(const double& p, const double& i, const double& d, const double& i_max, const double& i_min)
{
  for (unsigned int j = 0; j < 4; j++)
    pid_controller_[j].setGains(p, i, d, i_max, i_min);
}

void HeroChassisController::getGains(double& p, double& i, double& d, double& i_max, double& i_min)
{
  for (unsigned int j = 0; j < 4; j++)
    pid_controller_[j].getGains(p, i, d, i_max, i_min);
}

void HeroChassisController::starting(const ros::Time& time)
{
  for (unsigned int j = 0; j < 4; j++)
    pid_controller_[j].reset();
}

void HeroChassisController::update(const ros::Time& time, const ros::Duration& period)
{
  updateOdometry(time);
  //使用tf计算实现世界坐标下的速度控制,返回的是底盘坐标系下的速度指令
  if(odomcontrol == 1)
  {
    tf::TransformListener listener(ros::Duration(10));
    transformvector(listener);
  }
  double v_w[4] = { 0,0,0,0 };

  v_w[0] = (v_tx + v_ty - (r_x + r_y) * omiga) / r_wheel;
  v_w[1] = (v_tx - v_ty + (r_x + r_y) * omiga) / r_wheel;
  v_w[2] = (v_tx - v_ty - (r_x + r_y) * omiga) / r_wheel;
  v_w[3] = (v_tx + v_ty + (r_x + r_y) * omiga) / r_wheel;

  // Set the PID error and compute the PID command with nonuniform time and computed from the change in the error

  double error0 = v_w[0] - front_left_joint_.getVelocity();
  double error1 = v_w[1] - front_right_joint_.getVelocity();
  double error2 = v_w[2] - back_left_joint_.getVelocity();
  double error3 = v_w[3] - back_right_joint_.getVelocity();

  output[0] = pid_controller_[0].computeCommand(error0, period);
  output[1] = pid_controller_[1].computeCommand(error1, period);
  output[2] = pid_controller_[2].computeCommand(error2, period);
  output[3] = pid_controller_[3].computeCommand(error3, period);

  front_left_joint_.setCommand(output[0]);
  front_right_joint_.setCommand(output[1]);
  back_left_joint_.setCommand(output[2]);
  back_right_joint_.setCommand(output[3]);


}

void HeroChassisController::updateOdometry(const ros::Time& time)
{
  //获得速度
  double fl_speed = (front_left_joint_.getVelocity()) * r_wheel;
  double fr_speed = (front_right_joint_.getVelocity()) * r_wheel;
  double bl_speed = (back_left_joint_.getVelocity()) * r_wheel;
  double br_speed = (back_right_joint_.getVelocity()) * r_wheel;
//计算里程计
  v_x = (fl_speed + fr_speed) / 2;
  v_y = (bl_speed + br_speed) / 2;
  w = (br_speed - fl_speed) / 2 * (r_x + r_y);
  current_time = ros::Time::now();
  double dt = (current_time - last_time).toSec();
  last_time = current_time;

  double delta_x = (v_x * cos(th) - v_y * sin(th)) * dt;
  double delta_y = (v_x * sin(th) + v_y * cos(th)) * dt;
  double delta_th = w * dt;

  x += delta_x;
  y += delta_y;
  th += delta_th;

  //quaternion created from yaw
  tf2::Quaternion qtn;
  qtn.setRPY(0,0,th) ;

  // publish the transform over tf
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
   if(tf_odom_pub_->trylock()){
    //geometry_msgs::TransformStamped odom_frame= tf_odom_pub_->msg_.transforms[0];
    tf_odom_pub_->msg_.transforms[0].header.stamp = time;
    tf_odom_pub_->msg_.transforms[0].transform.translation.x = x;
    tf_odom_pub_->msg_.transforms[0].transform.translation.y = y;
    tf_odom_pub_->msg_.transforms[0].transform.rotation = odom_quat;
    tf_odom_pub_->unlockAndPublish();
  }
/*
  odomTf_.header.stamp = time;
  odomTf_.header.frame_id = odom_;
  odomTf_.child_frame_id = base_link_;
  odomTf_.transform.translation.x = x;
  odomTf_.transform.translation.y = y;
  odomTf_.transform.translation.z = 0;
  odomTf_.transform.rotation.x = qtn.x();
  odomTf_.transform.rotation.y = qtn.y();
  odomTf_.transform.rotation.z = qtn.z();
  odomTf_.transform.rotation.w = qtn.w();
  tfBroadcaster_.sendTransform(odomTf_);*/


  if (odom_pub_->trylock())
  {
    odom_pub_->msg_.header.stamp = current_time;
    odom_pub_->msg_.header.frame_id = "odom";
    odom_pub_->msg_.child_frame_id = "base_link";
    odom_pub_->msg_.pose.pose.position.x = x;
    odom_pub_->msg_.pose.pose.position.y = y;
    odom_pub_->msg_.pose.pose.position.z = 0.0;
    odom_pub_->msg_.pose.pose.orientation = odom_quat;
    // set the velocity
    odom_pub_->msg_.twist.twist.linear.x = v_x;
    odom_pub_->msg_.twist.twist.linear.y = v_y;
    odom_pub_->msg_.twist.twist.angular.z = w;
    odom_pub_->unlockAndPublish();
  }
}

void HeroChassisController::transformvector(const tf::TransformListener& listener){
  //we'll create a  frame  to transform to the base_link frame
  geometry_msgs::Vector3Stamped odom_cmd;
  odom_cmd.header.frame_id = "odom";
  odom_cmd.header.stamp = ros::Time();
  odom_cmd.vector.x = v_tx;
  odom_cmd.vector.y = v_ty;
  odom_cmd.vector.z = 0.0;

  try{
    geometry_msgs::Vector3Stamped base_cmd;
    listener.transformVector("base_link", odom_cmd, base_cmd);

    ROS_INFO("base_laser: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
             odom_cmd.vector.x, odom_cmd.vector.y, odom_cmd.vector.z,
             base_cmd.vector.x, base_cmd.vector.y, base_cmd.vector.z, base_cmd.header.stamp.toSec());

    v_tx = base_cmd.vector.x;
    v_ty = base_cmd.vector.y;

  }
  catch(tf::TransformException& ex){
    ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
  }
}
}
PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase)








