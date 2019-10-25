#include <ihmc_msgs/ArmDesiredAccelerationsRosMessage.h>
#include <tough_controller_interface/arm_control_interface.h>
#include "ros/ros.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_ihmc_msgs");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ArmControlInterface arm_controller(nh);
  std::vector<double> arm_values;
  arm_values.resize(7);
  for (size_t i = 0; i < 7; i++)
  {
    arm_values.at(i) = 2;
  }

  arm_controller.moveArmJointsAcceleration(RobotSide::LEFT, arm_values);
  ros::Duration(0.5).sleep();
  spinner.stop();

  return 0;
}
