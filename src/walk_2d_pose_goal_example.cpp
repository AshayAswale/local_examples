#include "ros/ros.h"
#include "tough_footstep/robot_walker.h"
#include <ihmc_msgs/FootstepDataListRosMessage.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/String.h>

RobotWalker* walking_controller;

void walkRobot(const geometry_msgs::PoseStamped& goal)
{
  tf::Quaternion rot_quat;
  tf::quaternionMsgToTF(goal.pose.orientation, rot_quat);

  tf::Matrix3x3 matx(rot_quat);

  double roll, pitch, yaw;
  matx.getRPY(roll, pitch, yaw);

  geometry_msgs::Pose2D msg_out;
  msg_out.x = goal.pose.position.x;
  msg_out.y = goal.pose.position.y;
  msg_out.theta = yaw;

  walking_controller->walkToGoal(msg_out);
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "walk_2d_pose_goal_example");
  ros::NodeHandle nh;
  walking_controller = new RobotWalker(nh, 0.4, 0.4);
  // ROS_INFO_THROTTLE(10,"Running.");
  ros::Subscriber subscribe = nh.subscribe("/goal", 1000, walkRobot);
  ros::spin();
}