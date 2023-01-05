#include <controller_interface/controller.h>
#include <effort_controllers/joint_velocity_controller.h>

namespace hero_chassis_controller {

class HeroChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
 public:
  HeroChassisController() = default;
  ~HeroChassisController() override = default;

  bool init(hardware_interface::EffortJointInterface *effort_joint_interface,
            ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh, const control_toolbox::Pid &pid) override;

  void update(const ros::Time &time, const ros::Duration &period) override;

  hardware_interface::JointHandle front_left_joint_, front_right_joint_, back_left_joint_, back_right_joint_;
 private:
  int state_{};
  ros::Time last_change_;
  ros::Time current_time, last_time;
  
};
}// namespace hero_chassis_controller 


