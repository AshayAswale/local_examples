#include <ihmc_msgs/ArmDesiredAccelerationsRosMessage.h>
#include "ros/ros.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_ihmc_msgs");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::string topic = "/ihmc_ros/atlas/control/arm_desired_joint_accelerations";
  // ros::Publisher acceleration_publisher(topic, 1000, ihmc_msgs::ArmDesiredAccelerationsRosMessage, nh);
  ros::Publisher acceleration_publisher = nh.advertise<ihmc_msgs::ArmDesiredAccelerationsRosMessage>(topic, 1, true);

  ihmc_msgs::ArmDesiredAccelerationsRosMessage arm_message;
  arm_message.robot_side = arm_message.LEFT;
  arm_message.unique_id = rand() % 100;
  arm_message.desired_joint_accelerations.resize(7);
  for (size_t i = 0; i < 7; i++)
  {
    arm_message.desired_joint_accelerations.at(i) = 2;
  }

  acceleration_publisher.publish(arm_message);
  ros::Duration(0.5).sleep();
  spinner.stop();

  return 0;
}
